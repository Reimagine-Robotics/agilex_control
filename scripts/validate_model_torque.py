"""THROWAWAY: model-predicted gravity torque vs agilex_control-measured torque.

Do not merge -- throwaway validation for the sysid-through-agilex-control plan.

Holds a pose with POSITION control (kp/kd, t_ff=0 -- so there is NO c on the
command path) and prints, per joint, the model's predicted gravity torque
(model.predict(q)) against the torque the arm actually reports via
get_motor_states (agilex/pyAgxArm units), plus their ratio. Because the hold
uses t_ff=0, the measured torque is pure gravity in agilex units -- a clean
reference the model can be checked against.

How to read it:
  - ratio ~1 on every loaded joint  -> the model is already consistent with
    agilex_control's torque units, so a model sysid'd THROUGH agilex_control
    would deploy raw (no c compensation). This validates the plan.
  - wrist ratio ~1/c (e.g. ~1.23 on a base Piper) -> the current
    piper_sdk-calibrated model is off by c, which is what --compensate-c
    patches, and which an agilex_control re-sysid would absorb into the masses.

SAFETY: the arm holds its startup pose under position control. Position it by
hand with the WRIST LOADED (j5 roughly horizontal, so gravity >> friction),
hold it, and let go after the countdown. Compare a few poses. Ctrl-C relaxes and
disables. Friction/stiction adds a small per-joint offset, so trust loaded
joints (j2, j5) over near-balanced ones.

To run:
  python3 scripts/validate_model_torque.py --model-path <arm.xml>
"""

# pylint: disable=protected-access

import argparse
import logging
import statistics
import time

import numpy as np

from agilex_control import agilex_interface, can_utils, gravity_compensation

logger = logging.getLogger(__name__)


def _measure_torques(arm, num_joints: int, samples: int = 8) -> np.ndarray:
  """Median per-joint measured torque (N.m, agilex units) over a few reads."""
  reads: list[list[float]] = [[] for _ in range(num_joints)]
  for _ in range(samples):
    for i in range(num_joints):
      state = arm._arm.get_motor_states(i + 1)
      if state is not None:
        reads[i].append(state.msg.torque)
    time.sleep(0.02)
  return np.array([statistics.median(r) if r else float("nan") for r in reads])


def main() -> None:
  logging.basicConfig(level=logging.INFO)
  parser = argparse.ArgumentParser(
      description=__doc__,
      formatter_class=argparse.RawDescriptionHelpFormatter,
  )
  parser.add_argument("--model-path", required=True, help="MuJoCo XML model.")
  parser.add_argument("--can-port", default="can0", help="CAN port.")
  parser.add_argument(
      "--arm-type",
      default=agilex_interface.ArmType.PIPER.name,
      choices=[arm_type.name for arm_type in agilex_interface.ArmType],
      help="Arm model.",
  )
  parser.add_argument("--kp", type=float, default=15.0, help="Hold kp.")
  parser.add_argument("--kd", type=float, default=1.0, help="Hold kd.")
  parser.add_argument(
      "--countdown",
      type=float,
      default=5.0,
      help="Seconds to position/hold the arm before it takes over the hold.",
  )
  args = parser.parse_args()

  arm_type = agilex_interface.ArmType[args.arm_type]
  can_utils.activate()
  logger.info("Connecting on %s ...", args.can_port)
  arm = agilex_interface.ArmInterface(can_port=args.can_port, arm_type=arm_type)
  num_joints = 0
  q_des = None  # Captured hold pose; None until the countdown completes.
  try:
    logger.info("Firmware: %s", arm.get_firmware_version())
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
        "\nPosition the arm by hand with the WRIST LOADED (j5 ~horizontal) and"
        " hold it. The arm stays LIMP during the countdown; after it, it holds"
        " that pose -- then let go."
    )
    # Hold zero torque so the arm is backdrivable while you position it. Some
    # firmwares hold stiff on enable with no command, which blocks hand motion.
    end = time.monotonic() + args.countdown
    next_print = args.countdown
    while time.monotonic() < end:
      for i in range(num_joints):
        arm.command_joint_torque_mit(i, 0.0)
      remaining = end - time.monotonic()
      if remaining <= next_print:
        print(f"  taking over hold in {int(remaining) + 1} ...")
        next_print -= 1
      time.sleep(0.02)
    q_des = np.array(arm.get_joint_positions())
    logger.info("Holding at q(deg)=%s", np.degrees(q_des))
    print("Let go. Comparing model vs measured torque. Ctrl-C to stop.\n")

    np.set_printoptions(precision=2, suppress=True, sign=" ")
    while True:
      # Hold the captured pose with pure PD (t_ff=0 -> no c on the command).
      for i in range(num_joints):
        arm.command_joint_position_mit(
            i, position=float(q_des[i]), kp=args.kp, kd=args.kd
        )
      try:
        q = np.array(arm.get_joint_positions())
      except RuntimeError:
        time.sleep(0.05)
        continue
      tau_model = np.asarray(model.predict(q))
      tau_meas = _measure_torques(arm, num_joints)
      with np.errstate(divide="ignore", invalid="ignore"):
        ratio = tau_model / tau_meas
      print(f"q(deg):  {np.degrees(q)}")
      print(f"model:   {tau_model}")
      print(f"measured:{tau_meas}")
      print(f"ratio:   {ratio}\n")
      time.sleep(0.4)
  except KeyboardInterrupt:
    print("\nStopping.")
  finally:
    # It does NOT park itself. Keep holding the pose for a few seconds so you
    # can support the arm before it goes limp -- otherwise it drops on Ctrl-C.
    if q_des is not None:
      print("Support the arm -- going limp in 3s...")
      end = time.monotonic() + 3.0
      while time.monotonic() < end:
        try:
          for i in range(num_joints):
            arm.command_joint_position_mit(
                i, position=float(q_des[i]), kp=args.kp, kd=args.kd
            )
        except (RuntimeError, ValueError, OSError):
          break
        time.sleep(0.02)
    logger.info("Relaxing and disabling...")
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
