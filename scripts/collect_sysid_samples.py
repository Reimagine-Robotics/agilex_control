"""THROWAWAY: collect sysid samples via agilex_control for the r2 fit.

Do not merge -- throwaway bridge for sysid-through-agilex-control validation.

Drives the arm to the poses r2's sysid_sim.generate_joint_samples produced
(deterministic D-optimal set, dumped to --poses-path), records averaged
(qpos, measured torque) at each settled pose via agilex_control's
get_motor_states, and saves an .npz with qpos/efforts/efforts_std -- exactly the
format r2's calibrate.py consumes via sysid_cfg.load_samples_path.

The ONLY differences from a native calibrate.py run are the torque source
(agilex get_motor_states instead of piper_control) and the motion driver
(agilex PD instead of joambot) -- same poses, same PD gains, same settle/sample
timing, same fit. So the fitted model lands in agilex_control torque units and
should deploy raw, with no c compensation.

SAFETY: the arm MOVES ITSELF through the poses. CLEAR THE WORKSPACE. The poses
are collision-filtered in the model but there is no real-world collision
routing. Ctrl-C holds the last pose briefly, then relaxes.

To run:
  # 1. in r2, dump the poses (see the r2 helper) -> poses.npy
  # 2. here:
  python3 scripts/collect_sysid_samples.py --poses-path poses.npy \
      --out-path samples.npz
  # 3. in r2: calibrate.py ... --cfg.sysid_cfg.load_samples_path=samples.npz
"""

# pylint: disable=protected-access

import argparse
import logging
import time

import numpy as np

from agilex_control import agilex_interface, can_utils

logger = logging.getLogger(__name__)

# Mirror r2 calibrate.py / sysid_sim.Config so only the torque source differs.
_KP_GAINS = (3.0, 3.0, 3.0, 3.0, 6.0, 4.0)
_KD_GAIN = 0.8
_SAMPLE_DURATION = 0.7  # s to average efforts at each settled pose.
_CONTROL_FREQ = 200.0  # Hz.
_SETTLE_VEL = 0.001  # rad/s; "settled" when every joint is below this.
_SETTLE_TIMEOUT = 3.0  # s to reach a pose before giving up on it.


def _measure_torques(arm, num_joints: int) -> np.ndarray:
  """One per-joint measured torque reading (N.m, agilex units)."""
  out = np.full(num_joints, np.nan)
  for i in range(num_joints):
    state = arm._arm.get_motor_states(i + 1)
    if state is not None:
      out[i] = state.msg.torque
  return out


def _hold(arm, pose: np.ndarray, num_joints: int) -> None:
  """Command one PD position-hold tick at pose (t_ff=0 -> no c on command)."""
  for i in range(num_joints):
    arm.command_joint_position_mit(
        i, position=float(pose[i]), kp=_KP_GAINS[i], kd=_KD_GAIN
    )


def _go_and_settle(arm, pose: np.ndarray, num_joints: int) -> bool:
  """Drive to pose and wait until settled (velocities < _SETTLE_VEL)."""
  time.sleep(0.5)  # initial settle after commanding, like calibrate.py.
  deadline = time.monotonic() + _SETTLE_TIMEOUT
  while time.monotonic() < deadline:
    _hold(arm, pose, num_joints)
    try:
      vel = np.array(arm.get_joint_velocities())
    except RuntimeError:
      time.sleep(0.01)
      continue
    if np.all(np.abs(vel) < _SETTLE_VEL):
      time.sleep(0.5)
      return True
    time.sleep(0.005)
  return False


def _record_sample(
    arm, pose: np.ndarray, num_joints: int
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
  """Average qpos/effort over _SAMPLE_DURATION while holding pose."""
  qpos_s: list[np.ndarray] = []
  eff_s: list[np.ndarray] = []
  end = time.monotonic() + _SAMPLE_DURATION
  while time.monotonic() < end:
    _hold(arm, pose, num_joints)
    try:
      q = np.array(arm.get_joint_positions())
    except RuntimeError:
      continue
    eff = _measure_torques(arm, num_joints)
    qpos_s.append(q)
    eff_s.append(eff)
    time.sleep(1.0 / _CONTROL_FREQ)
  qp = np.stack(qpos_s, axis=0)
  ef = np.stack(eff_s, axis=0)
  return qp.mean(axis=0), np.nanmean(ef, axis=0), np.nanstd(ef, axis=0)


def main() -> None:
  logging.basicConfig(level=logging.INFO)
  parser = argparse.ArgumentParser(
      description=__doc__,
      formatter_class=argparse.RawDescriptionHelpFormatter,
  )
  parser.add_argument(
      "--poses-path", required=True, help="npy of (N, num_joints) joint poses."
  )
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
  args = parser.parse_args()

  poses = np.load(args.poses_path)
  logger.info("Loaded %d poses from %s", len(poses), args.poses_path)

  arm_type = agilex_interface.ArmType[args.arm_type]
  can_utils.activate()
  arm = agilex_interface.ArmInterface(can_port=args.can_port, arm_type=arm_type)
  num_joints = 0
  last_pose = None
  try:
    num_joints = arm.get_num_joints()
    if poses.shape[1] != num_joints or len(_KP_GAINS) != num_joints:
      raise ValueError(
          f"poses have {poses.shape[1]} joints, arm has {num_joints},"
          f" gains have {len(_KP_GAINS)} -- they must match."
      )
    arm.set_installation_pos(agilex_interface.ArmInstallationPos.UPRIGHT)

    print(
        f"\nWARNING: the arm will MOVE ITSELF through {len(poses)} poses."
        " Clear the workspace."
    )
    input("Press Enter to start... (Ctrl-C to stop)")

    logger.info("Enabling arm...")
    deadline = time.monotonic() + 5.0
    while not arm.enable_arm():
      if time.monotonic() >= deadline:
        raise TimeoutError("Timed out enabling the arm.")
      time.sleep(0.1)
    arm.set_mit_mode()

    qpos_buf: list[np.ndarray] = []
    eff_buf: list[np.ndarray] = []
    std_buf: list[np.ndarray] = []
    for idx, pose in enumerate(poses):
      last_pose = pose
      if not _go_and_settle(arm, pose, num_joints):
        logger.warning("Pose %d/%d timed out; skipping.", idx + 1, len(poses))
        continue
      mean_q, mean_eff, std_eff = _record_sample(arm, pose, num_joints)
      qpos_buf.append(mean_q)
      eff_buf.append(mean_eff)
      std_buf.append(std_eff)
      logger.info(
          "Pose %d/%d recorded (effort |max|=%.2f)",
          idx + 1,
          len(poses),
          np.nanmax(np.abs(mean_eff)),
      )

    np.savez(
        args.out_path,
        qpos=np.stack(qpos_buf, axis=0),
        efforts=np.stack(eff_buf, axis=0),
        efforts_std=np.stack(std_buf, axis=0),
    )
    logger.info("Saved %d samples to %s", len(qpos_buf), args.out_path)
  except KeyboardInterrupt:
    print("\nStopping.")
  finally:
    # Hold a moment so the arm can be supported before it goes limp.
    if last_pose is not None:
      print("Support the arm -- going limp in 2s...")
      end = time.monotonic() + 2.0
      while time.monotonic() < end:
        try:
          _hold(arm, last_pose, num_joints)
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
