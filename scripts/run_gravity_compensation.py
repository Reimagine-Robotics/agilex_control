"""CLI script for running gravity compensation on AgileX arms.

This activates a backdrivable teach-mode by feeding the MuJoCo gravity model as
pure MIT feed-forward torque (kp=kd=0) plus a small per-joint velocity damping
term, so the arm holds against gravity but moves freely when pushed by hand.

It assumes the MuJoCo model has already been calibrated in the arm's own torque
units, so its predicted gravity torque can be commanded directly with no extra
scaling. An uncalibrated or mismatched model (e.g. raw CAD masses) will sag or
float.

Requires the ``gravity`` extra for MuJoCo (e.g. ``uv sync --extra gravity``);
without it the import fails before argument parsing.

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
    agilex_init,
    agilex_interface,
    can_utils,
    gravity_compensation,
)

logger = logging.getLogger(__name__)

# Per-joint velocity damping, applied as software feed-forward torque
# (-qvel * gain) so the arm stays freely backdrivable. Keyed by arm type.
_TEACH_DGAIN_BY_ARM_TYPE = {
    agilex_interface.ArmType.PIPER: (0.0, 0.002, 0.002, 0.018, 0.018, 0.018),
    agilex_interface.ArmType.PIPER_H: (0.0, 0.002, 0.002, 0.018, 0.018, 0.018),
    agilex_interface.ArmType.PIPER_X: (0.0, 0.002, 0.002, 0.018, 0.018, 0.018),
    agilex_interface.ArmType.PIPER_L: (0.0, 0.002, 0.002, 0.018, 0.018, 0.018),
    agilex_interface.ArmType.NERO: (0.0, 0.05, 0.03, 0.05, 0.01, 0.02, 0.01),
}


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
          "Multiply the model torque by the per-joint c coefficient. Set this"
          " only for backwards compatibility with a model calibrated against"
          " k*b effort (piper_control), not k*b*c torque (agilex_control);"
          " leave off for an agilex_control-calibrated model."
      ),
  )
  parser.add_argument(
      "--compensate-c-damping",
      action="store_true",
      help=(
          "Multiply the velocity-damping term by the per-joint c coefficient."
          " The dgain values are inherited from piper_control, whose k*b effort"
          " command was not c-divided; agilex_control's driver divides every"
          " command by c, so an uncorrected dgain damps ~1/c harder (e.g. ~1.23x"
          " at the base-Piper wrist, c=0.813). Enable this to cancel that and"
          " reproduce piper_control's physical damping. Off by default; use it"
          " to A/B against per-arm-type tuning of the raw dgain."
      ),
  )
  args = parser.parse_args()

  arm_type = agilex_interface.ArmType[args.arm_type]

  can_utils.activate()
  logger.info("Connecting on %s ...", args.can_port)
  arm = agilex_interface.ArmInterface(can_port=args.can_port, arm_type=arm_type)
  try:
    logger.info("Firmware version: %s", arm.get_firmware_version())

    # Default the joint names to joint1..jointN from the arm's joint count.
    joint_names = args.joint_names or [
        f"joint{i}" for i in range(1, arm.get_num_joints() + 1)
    ]
    if len(joint_names) != arm.get_num_joints():
      raise ValueError(
          f"--joint-names has {len(joint_names)} name(s) but the arm has"
          f" {arm.get_num_joints()} joints; they must map 1:1 (otherwise one"
          " joint's torque broadcasts across all motors)."
      )
    logger.info("Loading gravity compensation model...")
    model = gravity_compensation.GravityCompensationModel(
        model_path=args.model_path, joint_names=joint_names
    )

    # Prepare the two per-joint corrections to the model torque:
    # 1) command_scale: the k*b scale (see direct_scaling_factors) for non-base
    # Piper arms, computed before enabling so unsupported firmware raises first.
    # 2) torque_c: the per-joint torque c coefficient, applied only for
    # backwards compatibility with a model calibrated against k*b effort
    # (piper_control), not k*b*c torque (agilex_control).
    command_scale = np.array(arm.direct_scaling_factors())
    logger.info("command scale: %s", command_scale)
    _, _, joint_torque_c = arm.get_joint_torque_coefficients()
    torque_c = np.array(joint_torque_c) if args.compensate_c else 1.0
    # Independent of torque_c: the dgain is piper_control-inherited (k*b effort),
    # so c-correcting the damping reproduces its physical feel on the driver's
    # c-divided command path. Keyed off the dgain's origin, not the model's.
    damping_c = np.array(joint_torque_c) if args.compensate_c_damping else 1.0

    logger.info("enabling arm")
    agilex_init.enable_arm(arm)

    shutdown = threading.Event()

    def _handle_signal(signum, frame):
      del signum, frame
      shutdown.set()

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    dgain = np.array(_TEACH_DGAIN_BY_ARM_TYPE[arm_type])

    with agilex_control.MitJointPositionController(
        arm,
        kp_gains=5.0,  # Unused by command_torques; only for the stop() park.
        kd_gains=0.8,
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
        hover_torque = model.predict(qpos) * command_scale * torque_c
        stability_torque = -qvel * dgain * damping_c
        command = hover_torque + stability_torque
        controller.command_torques(command.tolist())
        time.sleep(0.005)
    # Leaving the controller context parks the arm at its rest pose and relaxes.
  finally:
    logger.info("Cleaning up...")
    try:
      agilex_init.disable_arm(arm)
    finally:
      arm.disconnect()
    logger.info("Done.")


if __name__ == "__main__":
  main()
