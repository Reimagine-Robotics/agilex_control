"""CLI script for backdrivable teach-mode gravity compensation.

Mirrors r2's TeachController: feeds the MuJoCo gravity model as pure MIT
feed-forward torque (kp=kd=0) plus a small per-joint velocity damping term, so
the arm holds against gravity but moves freely when pushed by hand.

--compensate-c pre-scales the commanded torque by the per-joint torque
coefficient c (read from the arm). The base-Piper setup needs it (the raw wrist
over-delivers by ~1/c and floats); piperH does NOT (raw is correct there, and
this makes it sag), so it is opt-in until the arm/model split is understood. Use
a calibrated (live) model matched to the arm; a wrong/too-light model sags and a
CAD model over-estimates and floats, independent of c. On newer firmware the
gravity torque can also exceed the t_ff register limit (±8 N·m) at extended
poses.

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
      "--damping-scale",
      type=float,
      default=1.0,
      help=(
          "Scale the teach velocity damping (0 = off). Diagnostic: a joint that"
          " floats with damping but holds at 0 has a flipped velocity sign"
          " (-qvel*dgain becomes anti-damping)."
      ),
  )
  parser.add_argument(
      "--compensate-c",
      action="store_true",
      help=(
          "Pre-multiply commanded torque by the per-joint torque coefficient c"
          " (read from the arm). Needed on the base-Piper setup (raw wrist"
          " floats); NOT on piperH (raw is correct there, and this makes it"
          " sag) -- so it is opt-in until we understand the arm/model split."
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
    # --damping-scale scales it (0 = no damping) -- a diagnostic: the damping is
    # -qvel*dgain, so a flipped joint-velocity sign turns it into anti-damping
    # (drives that joint). If a joint floats with damping but holds at
    # --damping-scale 0, its reported velocity sign is flipped.
    dgain = args.damping_scale * (
        np.array(_TEACH_DGAIN)
        if arm.get_num_joints() == len(_TEACH_DGAIN)
        else 0.018
    )

    # pyAgxArm's move_mit divides commanded t_ff by the per-joint torque
    # coefficient c but the arm doesn't restore it (see command_joint_torque_mit
    # / get_joint_torque_coefficients), so the wrist over-delivers by ~1/c and
    # floats. Pre-multiply the command by c to cancel it. Read from the arm, so
    # it's correct for whatever arm type this is (PIPER, PIPER_H, ...). Keep
    # --no-compensate-c to A/B the raw (floating) behaviour.
    torque_scale = 1.0
    if args.compensate_c:
      torque_scale = np.array(arm.get_joint_torque_coefficients())
      logger.info("compensate-c ON: scaling command by c=%s", torque_scale)

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
