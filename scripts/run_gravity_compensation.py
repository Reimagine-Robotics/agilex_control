"""CLI script for gravity-compensated position hold, mirroring r2's serve.

Holds the arm at the pose it is engaged in using position control (kp/kd) with
the MuJoCo gravity model fed as MIT feed-forward torque. This matches how r2's
run_serve_piper runs gravity compensation: the (un-clamped) position servo holds
the arm and the gravity feed-forward cancels most of the load so it feels light.
Pure feed-forward instead -- the gravity torque as the only holding force --
saturates the firmware's t_ff register (±8 N·m on newer firmware) at extended
poses and can't hold the arm.

Push the arm and it returns to the held pose, gravity-compensated. kd is the
motor's native in-loop damping, not a software term.

To run:
  python3 scripts/run_gravity_compensation.py --model-path <path/to/arm.xml>
"""

import argparse
import logging
import signal
import threading
import time

from agilex_control import (
    agilex_control,
    agilex_interface,
    can_utils,
    gravity_compensation,
)

logger = logging.getLogger(__name__)

# Position and damping gains, matching r2's run_serve_piper DEFAULT_KP/KD_GAINS
# (base Piper, 6 joints). The position servo does the holding; gravity
# feed-forward cancels most of the load. kd is the motor's native damping.
_KP_GAINS = (5.5, 5.5, 10.0, 15.0, 25.0, 15.0)
_KD_GAIN = 0.8


def _wait_for_joint_positions(
    arm: agilex_interface.ArmInterface, timeout: float = 5.0
) -> list[float]:
  """Returns the first available joint position reading (retries on no data)."""
  deadline = time.monotonic() + timeout
  while True:
    try:
      return arm.get_joint_positions()
    except RuntimeError:
      if time.monotonic() >= deadline:
        raise
      time.sleep(0.05)


def main() -> None:
  logging.basicConfig(level=logging.INFO)
  parser = argparse.ArgumentParser(
      description=__doc__,
      formatter_class=argparse.RawDescriptionHelpFormatter,
  )
  parser.add_argument(
      "--model-path",
      required=True,
      help="Path to the MuJoCo XML model (required; no bundled default).",
  )
  parser.add_argument(
      "--joint-names",
      nargs="+",
      default=None,
      help=(
          "Joint names in the MuJoCo model. Defaults to joint1..jointN derived"
          " from the arm's joint count."
      ),
  )
  parser.add_argument("--can-port", default="can0", help="CAN port.")
  parser.add_argument(
      "--arm-type",
      default=agilex_interface.ArmType.PIPER.name,
      choices=[arm_type.name for arm_type in agilex_interface.ArmType],
      help="Arm model.",
  )
  args = parser.parse_args()

  arm_type = agilex_interface.ArmType[args.arm_type]

  can_utils.activate()
  logger.info("Connecting on %s ...", args.can_port)
  arm = agilex_interface.ArmInterface(can_port=args.can_port, arm_type=arm_type)
  try:
    logger.info("Firmware version: %s", arm.get_firmware_version())

    # Default the joint names to joint1..jointN from the arm's joint count
    # (pyAgxArm uses this convention for every model, including the 7-joint
    # Nero), rather than hardcoding six.
    joint_names = args.joint_names or [
        f"joint{i}" for i in range(1, arm.get_num_joints() + 1)
    ]
    logger.info("Loading gravity compensation model...")
    model = gravity_compensation.GravityCompensationModel(
        model_path=args.model_path, joint_names=joint_names
    )

    arm.set_installation_pos(agilex_interface.ArmInstallationPos.UPRIGHT)

    # Enable the motors, retrying until enabled (stand-in until agilex_init).
    logger.info("enabling arm")
    deadline = time.monotonic() + 5.0
    while not arm.enable_arm():
      if time.monotonic() >= deadline:
        raise TimeoutError("Timed out enabling the arm.")
      time.sleep(0.1)

    shutdown = threading.Event()

    def _handle_signal(signum, frame):
      del signum, frame
      shutdown.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    # r2's gains assume a 6-joint base Piper; fall back to a scalar otherwise.
    kp_gains = _KP_GAINS if arm.get_num_joints() == len(_KP_GAINS) else 10.0

    with agilex_control.MitJointPositionController(
        arm,
        kp_gains=kp_gains,
        kd_gains=_KD_GAIN,
        rest_position=agilex_control.ArmOrientations.upright.rest_position,
    ) as controller:
      logger.info("Starting gravity compensation mode...")
      input("Press Enter to start... (Ctrl-C to stop)")
      # Hold the pose the arm is in now; the position servo + gravity
      # feed-forward keep it there.
      hold_target = _wait_for_joint_positions(arm)
      while not shutdown.is_set():
        try:
          qpos = arm.get_joint_positions()
        except RuntimeError:
          # No feedback this tick; skip it (the motor holds its last command).
          time.sleep(0.005)
          continue
        gravity_torque = model.predict(qpos).tolist()
        controller.command_joints(hold_target, torques_ff=gravity_torque)
        time.sleep(0.005)
    # Leaving the controller context parks the arm at its rest pose and relaxes.
  finally:
    logger.info("Cleaning up...")
    try:
      arm.disable_arm()
    finally:
      arm.disconnect()
    logger.info("Done.")


if __name__ == "__main__":
  main()
