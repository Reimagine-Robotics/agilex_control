"""CLI script for backdrivable teach-mode gravity compensation.

Mirrors r2's TeachController: feeds the MuJoCo gravity model as pure MIT
feed-forward torque (kp=kd=0) plus a small per-joint velocity damping term, so
the arm holds against gravity but moves freely when pushed by hand.

NOTE: this uses the raw MuJoCo model, which over-estimates mass on the arm, so
expect some residual float / over-compensation (the same as r2's teach mode) --
a calibrated model is the follow-up. On newer firmware the gravity torque can
also exceed the t_ff register limit (±8 N·m) at extended poses.

To run:
  python3 scripts/run_gravity_compensation.py --model-path <path/to/arm.xml>
"""

import argparse
import logging
import signal
import threading
import time

import numpy as np

from agilex_control import (
    agilex_control,
    agilex_interface,
    can_utils,
    gravity_compensation,
)

logger = logging.getLogger(__name__)

# Per-joint velocity damping, matching r2's TeachController _TEACH_DGAIN (base
# Piper, 6 joints): tiny on the big joints, zero on the base, so the arm stays
# freely backdrivable. Applied as software feed-forward torque (-qvel * gain).
_TEACH_DGAIN = (0.0, 0.002, 0.002, 0.018, 0.018, 0.018)

# Per-joint torque coefficient c for a base Piper, from pyAgxArm's
# ROBOT_JOINT_TORQUE_C (api/constants.py). Only the wrist joints differ from 1.
# pyAgxArm's v183/post6 move_mit does `t_ff /= c` before packing, assuming the
# firmware multiplies c back; if the firmware does NOT, the wrist gets ~1/c
# (1.23x) the commanded torque. j5 is the only wrist joint bearing a real
# gravity load, so that surplus floats j5 and nothing else -- the observed bug.
# --compensate-c pre-multiplies the command by c to cancel the SDK's divide, as
# an A/B test of that hypothesis on hardware.
_PIPER_TORQUE_C = (1.0, 1.0, 1.0, 0.813252, 0.813252, 0.813252)


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
  parser.add_argument(
      "--compensate-c",
      action="store_true",
      help=(
          "DIAGNOSTIC: pre-multiply commanded torque by the per-joint torque"
          " coefficient c to cancel pyAgxArm's t_ff/=c on the wrist. Use to"
          " A/B whether the j5 float is the c round-trip (base Piper only)."
      ),
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

    # Per-joint damping assumes a 6-joint base Piper; scalar fallback otherwise.
    dgain = (
        np.array(_TEACH_DGAIN)
        if arm.get_num_joints() == len(_TEACH_DGAIN)
        else 0.018
    )

    # DIAGNOSTIC: optional per-joint command scale to cancel pyAgxArm's t_ff/=c
    # (base Piper, 6 joints only). Off by default -> scale of 1.0 (no change).
    torque_scale = 1.0
    if args.compensate_c:
      if arm.get_num_joints() != len(_PIPER_TORQUE_C):
        raise ValueError("--compensate-c assumes a 6-joint base Piper.")
      torque_scale = np.array(_PIPER_TORQUE_C)
      logger.info("compensate-c ON: scaling command by %s", _PIPER_TORQUE_C)

    with agilex_control.MitJointPositionController(
        arm,
        kp_gains=5.0,  # Unused by command_torques; only for the stop() park.
        kd_gains=0.8,
        rest_position=agilex_control.ArmOrientations.upright.rest_position,
    ) as controller:
      logger.info("Starting gravity compensation mode...")
      input("Press Enter to start... (Ctrl-C to stop)")
      while not shutdown.is_set():
        try:
          qpos = arm.get_joint_positions()
          qvel = np.array(arm.get_joint_velocities())
        except RuntimeError:
          # No feedback this tick; skip it (the motor holds its last torque).
          time.sleep(0.005)
          continue
        hover_torque = model.predict(qpos)
        # Small per-joint damping so the arm stays backdrivable but settles.
        stability_torque = -qvel * dgain
        command = (hover_torque + stability_torque) * torque_scale
        controller.command_torques(command.tolist())
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
