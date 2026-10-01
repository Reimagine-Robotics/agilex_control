"""Hand-guided sysid sample collection via agilex_control.

Do not merge -- throwaway bridge for sysid-through-agilex-control, hand-guided
variant for tight spaces where auto-driving is unsafe.

You hand-position the arm at each pose (so you steer around the table, camera
mounts, etc.); it then holds that pose with GRAVITY FEED-FORWARD (so it stays
where you placed it and does NOT droop), you let go, and it records averaged
(qpos, measured torque) in agilex_control units. Saves the qpos/efforts/
efforts_std .npz that r2's calibrate.py consumes via load_samples_path -- so the
fit is identical to a normal sysid, just with hand-chosen poses and
agilex-measured torque.

Pick ~15-20 diverse LOADED poses: shoulder out at various heights, elbow bent,
wrist roughly horizontal. Skip near-vertical / balanced configs that carry no
gravity signal. During positioning it prints the model's predicted torque so you
can aim for loaded poses (big |tau|).

Why gravity feed-forward matters: a raw PD hold at low kp droops tens of degrees
under gravity (that is what made the auto-drive collector clip the table). With
feed-forward the arm holds accurately where you put it; the measured torque is
still pure gravity (total delivered torque balances gravity regardless of how it
splits between feed-forward and PD).

SAFETY: between poses the arm is LIMP -- you move it by hand. During a hold it
actively holds the captured pose; keep clear and let go once it takes over.
Ctrl-C saves what was collected, then relaxes.

To run:
  python3 scripts/collect_sysid_hand.py --model-path <arm.xml> \
      --out-path samples.npz
"""

# pylint: disable=protected-access

import argparse
import logging
import time

import numpy as np

from agilex_control import agilex_interface, can_utils, gravity_compensation

logger = logging.getLogger(__name__)

# Match r2 calibrate.py's position-hold gains; with gravity feed-forward these
# track accurately (feed-forward carries gravity, PD only the residual).
_KP_GAINS = (3.0, 3.0, 3.0, 3.0, 6.0, 4.0)
_KD_GAIN = 0.8
_SAMPLE_DURATION = 0.7  # s to average per pose (matches sysid_sim.Config).
_CONTROL_FREQ = 200.0
_RELEASE_GRACE = 1.5  # s holding before recording, so you can let go.


def _measure_torques(arm, num_joints: int) -> np.ndarray:
  """One per-joint measured torque reading (N.m, agilex units)."""
  out = np.full(num_joints, np.nan)
  for i in range(num_joints):
    state = arm._arm.get_motor_states(i + 1)
    if state is not None:
      out[i] = state.msg.torque
  return out


def _hold_ff(arm, q_des: np.ndarray, model, num_joints: int) -> np.ndarray:
  """One position-hold tick with gravity feed-forward; returns current qpos."""
  q = np.array(arm.get_joint_positions())
  ff = np.asarray(model.predict(q))
  for i in range(num_joints):
    arm.command_joint_position_mit(
        i,
        position=float(q_des[i]),
        kp=_KP_GAINS[i],
        kd=_KD_GAIN,
        torque_ff=float(ff[i]),
    )
  return q


def main() -> None:
  logging.basicConfig(level=logging.INFO)
  parser = argparse.ArgumentParser(
      description=__doc__,
      formatter_class=argparse.RawDescriptionHelpFormatter,
  )
  parser.add_argument("--model-path", required=True, help="MuJoCo XML model.")
  parser.add_argument(
      "--out-path",
      required=True,
      help="Output .npz (qpos/efforts/efforts_std).",
  )
  parser.add_argument("--can-port", default="can0", help="CAN port.")
  parser.add_argument(
      "--arm-type",
      default=agilex_interface.ArmType.PIPER.name,
      choices=[arm_type.name for arm_type in agilex_interface.ArmType],
      help="Arm model.",
  )
  parser.add_argument(
      "--countdown",
      type=float,
      default=4.0,
      help="Seconds to position each pose before the arm takes over the hold.",
  )
  args = parser.parse_args()

  arm_type = agilex_interface.ArmType[args.arm_type]
  can_utils.activate()
  arm = agilex_interface.ArmInterface(can_port=args.can_port, arm_type=arm_type)
  num_joints = 0
  q_des = None
  model = None
  qpos_buf: list[np.ndarray] = []
  eff_buf: list[np.ndarray] = []
  std_buf: list[np.ndarray] = []
  np.set_printoptions(precision=2, suppress=True, sign=" ")
  try:
    num_joints = arm.get_num_joints()
    joint_names = [f"joint{i}" for i in range(1, num_joints + 1)]
    model = gravity_compensation.GravityCompensationModel(
        model_path=args.model_path, joint_names=joint_names
    )
    arm.set_installation_pos(agilex_interface.ArmInstallationPos.UPRIGHT)
    logger.info("Enabling arm...")
    deadline = time.monotonic() + 5.0
    while not arm.enable_arm():
      if time.monotonic() >= deadline:
        raise TimeoutError("Timed out enabling the arm.")
      time.sleep(0.1)
    arm.set_mit_mode()
    print(
        "\nHand-guided sysid collection. For each pose: move the arm by hand"
        " (it's limp) to a LOADED pose, hold it, wait for takeover, let go."
        " Ctrl-C when done (saves)."
    )

    while True:
      # Positioning phase: limp, show predicted torque so you aim for load.
      end = time.monotonic() + args.countdown
      next_print = args.countdown
      while time.monotonic() < end:
        for i in range(num_joints):
          arm.command_joint_torque_mit(i, 0.0)  # limp / backdrivable.
        try:
          q = np.array(arm.get_joint_positions())
          tau = np.asarray(model.predict(q))
        except RuntimeError:
          time.sleep(0.02)
          continue
        remaining = end - time.monotonic()
        if remaining <= next_print:
          print(
              f"  pose {len(qpos_buf) + 1}: hold still, takeover in"
              f" {int(remaining) + 1}s  model tau={tau}"
          )
          next_print -= 1
        time.sleep(0.02)

      q_des = np.array(arm.get_joint_positions())
      print(f"  takeover at q(deg)={np.degrees(q_des)} -- LET GO.")

      # Hold with gravity feed-forward; grace period so you can release.
      grace_end = time.monotonic() + _RELEASE_GRACE
      while time.monotonic() < grace_end:
        _hold_ff(arm, q_des, model, num_joints)
        time.sleep(1.0 / _CONTROL_FREQ)

      # Record averaged qpos/effort while holding.
      qpos_s: list[np.ndarray] = []
      eff_s: list[np.ndarray] = []
      rec_end = time.monotonic() + _SAMPLE_DURATION
      while time.monotonic() < rec_end:
        q = _hold_ff(arm, q_des, model, num_joints)
        eff_s.append(_measure_torques(arm, num_joints))
        qpos_s.append(q)
        time.sleep(1.0 / _CONTROL_FREQ)

      qpos_buf.append(np.stack(qpos_s, axis=0).mean(axis=0))
      eff_buf.append(np.nanmean(np.stack(eff_s, axis=0), axis=0))
      std_buf.append(np.nanstd(np.stack(eff_s, axis=0), axis=0))
      logger.info(
          "Recorded pose %d (effort |max|=%.2f). Reposition or Ctrl-C.",
          len(qpos_buf),
          np.nanmax(np.abs(eff_buf[-1])),
      )
  except KeyboardInterrupt:
    print("\nFinishing.")
  finally:
    if qpos_buf:
      np.savez(
          args.out_path,
          qpos=np.stack(qpos_buf, axis=0),
          efforts=np.stack(eff_buf, axis=0),
          efforts_std=np.stack(std_buf, axis=0),
      )
      logger.info("Saved %d samples to %s", len(qpos_buf), args.out_path)
    else:
      logger.warning("No samples collected; nothing saved.")
    # Hold a moment so you can support the arm before it goes limp.
    if q_des is not None and model is not None:
      print("Support the arm -- going limp in 2s...")
      end = time.monotonic() + 2.0
      while time.monotonic() < end:
        try:
          _hold_ff(arm, q_des, model, num_joints)
        except (RuntimeError, ValueError, OSError):
          break
        time.sleep(0.02)
    try:
      for i in range(num_joints):
        arm.command_joint_torque_mit(i, 0.0)
    except (RuntimeError, ValueError, OSError) as exc:
      logger.warning("Could not zero torque on exit: %s", exc)
    try:
      arm.disable_arm()
    finally:
      arm.disconnect()
    logger.info("Done.")


if __name__ == "__main__":
  main()
