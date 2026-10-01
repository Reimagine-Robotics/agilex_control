"""Probe the MIT feed-forward torque command->delivered scaling, per joint.

Commands a known pure feed-forward torque (kp=kd=0) on ONE joint while you HOLD
IT STILL by hand, then reads back the measured physical torque
(get_motor_states().msg.torque) and prints commanded vs measured and their
ratio. A ratio far from 1 means the command path is not delivering the N.m you
ask for.

Why this matters here: pyAgxArm's move_mit divides t_ff by the per-joint torque
coefficient c before packing, assuming the firmware multiplies c back. If the
firmware does NOT, the delivered (and hence measured) torque comes out ~1/c of
the command. For a base Piper the wrist has c=0.813, so the ratio reads ~1.23
there and ~1.0 on the base joints -- the signature of the j5 float. piperH has a
different c per joint (incl. c>1 on j6, i.e. ratio<1), so the measured ratios
are the clean way to compare arms.

For reference it prints the SDK's own c for the joint (read from the driver
config) and the expected 1/c, so the measured ratio can be checked directly.
This works for any arm type since c is read from the connected arm.

SAFETY: the joint is driven open-loop (no position/velocity feedback, no
damping). Keep the torques small, HOLD the joint firmly at a comfortable pose,
and be ready for it to move if you let go. It commands 0 between each step and
disables on exit.

To run:
  python3 scripts/probe_torque_scale.py --joint 5 --torques 0.5 1.0 1.5
"""

# pylint: disable=protected-access

import argparse
import logging
import statistics
import time

from agilex_control import agilex_interface, can_utils

logger = logging.getLogger(__name__)


def _measure_torque(arm, joint_1based: int, samples: int = 10) -> float:
  """Median of a few get_motor_states torque reads for one joint (N.m)."""
  reads = []
  for _ in range(samples):
    state = arm._arm.get_motor_states(joint_1based)
    if state is not None:
      reads.append(state.msg.torque)
    time.sleep(0.03)
  if not reads:
    raise RuntimeError(f"No motor-state feedback for joint {joint_1based}.")
  return statistics.median(reads)


def main() -> None:
  logging.basicConfig(level=logging.INFO)
  parser = argparse.ArgumentParser(
      description=__doc__,
      formatter_class=argparse.RawDescriptionHelpFormatter,
  )
  parser.add_argument(
      "--joint",
      type=int,
      required=True,
      help="1-based joint index to probe (hold THIS joint still).",
  )
  parser.add_argument(
      "--torques",
      type=float,
      nargs="+",
      default=[0.5, 1.0, 1.5],
      help="Feed-forward torques (N.m) to command in sequence.",
  )
  parser.add_argument("--can-port", default="can0", help="CAN port.")
  parser.add_argument(
      "--arm-type",
      default=agilex_interface.ArmType.PIPER.name,
      choices=[arm_type.name for arm_type in agilex_interface.ArmType],
      help="Arm model.",
  )
  parser.add_argument(
      "--settle",
      type=float,
      default=0.5,
      help="Seconds to hold each commanded torque before measuring.",
  )
  args = parser.parse_args()

  arm_type = agilex_interface.ArmType[args.arm_type]
  joint = args.joint  # 1-based for get_motor_states / display.

  can_utils.activate()
  logger.info("Connecting on %s ...", args.can_port)
  arm = agilex_interface.ArmInterface(can_port=args.can_port, arm_type=arm_type)
  try:
    logger.info("Firmware: %s", arm.get_firmware_version())
    if not 1 <= joint <= arm.get_num_joints():
      raise ValueError(
          f"--joint {joint} out of range 1..{arm.get_num_joints()}."
      )

    # Per-joint torque coefficient c the driver divides t_ff by (read from the
    # SDK config, so this is exact for whatever arm type is connected).
    c_table = arm._arm._config.get("joint_torque_c")
    c = c_table[joint - 1] if c_table else 1.0
    logger.info(
        "Joint %d: c=%.5f  -> expected measured/commanded ~ 1/c = %.3f",
        joint,
        c,
        1.0 / c,
    )

    logger.info("Enabling arm...")
    deadline = time.monotonic() + 5.0
    while not arm.enable_arm():
      if time.monotonic() >= deadline:
        raise TimeoutError("Timed out enabling the arm.")
      time.sleep(0.1)
    arm.set_mit_mode()

    print(
        f"\nHOLD joint {joint} STILL by hand at a comfortable pose. It will be"
        f" commanded {args.torques} N.m in turn (0 in between)."
    )
    input("Press Enter to start... (Ctrl-C to stop)")

    print("\n commanded    measured    ratio")
    for t_cmd in args.torques:
      arm.command_joint_torque_mit(joint - 1, t_cmd)
      time.sleep(args.settle)
      t_meas = _measure_torque(arm, joint)
      arm.command_joint_torque_mit(joint - 1, 0.0)  # relax between steps.
      time.sleep(0.2)
      ratio = t_meas / t_cmd if t_cmd != 0 else float("nan")
      print(f"{t_cmd:>10.3f}  {t_meas:>10.3f}  {ratio:>7.3f}")
  finally:
    logger.info("Cleaning up (zeroing torque, disabling)...")
    try:
      arm.command_joint_torque_mit(joint - 1, 0.0)
    except (RuntimeError, ValueError, OSError) as exc:  # best-effort.
      logger.warning("Could not zero joint torque on exit: %s", exc)
    try:
      arm.disable_arm()
    finally:
      arm.disconnect()
    logger.info("Done.")


if __name__ == "__main__":
  main()
