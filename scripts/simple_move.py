"""Example of moving the arm a small amount and opening/closing the gripper.

To run this example:
python3 scripts/simple_move.py [--arm_type piper|piper_h|nero]
"""

import argparse
import time

from agilex_control import agilex_control, agilex_init, agilex_interface, can_utils

# How far to nudge the 2nd-to-last joint, in radians.
_MOVE_DELTA = 0.2

# Arm models with a rest pose to park at on exit. Models without one (Piper X,
# Piper L) would relax wherever they stop, so this example does not offer them.
_SUPPORTED_ARM_TYPES = (
    agilex_interface.ArmType.PIPER,
    agilex_interface.ArmType.PIPER_H,
    agilex_interface.ArmType.NERO,
)


def main() -> None:
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument(
      "--arm_type",
      type=str,
      default="piper",
      choices=[t.name.lower() for t in _SUPPORTED_ARM_TYPES],
      help="Model of arm to control.",
  )
  args = parser.parse_args()
  arm_type = agilex_interface.ArmType[args.arm_type.upper()]

  print(
      "This script will move the 2nd-to-last joint of the arm a small amount."
  )
  input("WARNING: the robot will move. Press Enter to continue...")

  can_utils.activate()
  ports = can_utils.active_ports()
  if not ports:
    raise ValueError(
        "No active CAN ports found. Make sure the arm is connected and on."
    )
  print(f"Using CAN port: {ports[0]}")

  arm = agilex_interface.ArmInterface(can_port=ports[0], arm_type=arm_type)
  try:
    # Enable the motors; these retry until enabled and raise TimeoutError
    # otherwise.
    print("enabling arm")
    agilex_init.enable_arm(arm)
    agilex_init.enable_gripper(arm)

    # Open then close the gripper (commanding it enables it implicitly).
    print("testing gripper: open then close")
    with agilex_control.GripperController(arm) as gripper:
      gripper.command_open()
      time.sleep(2.0)
      gripper.command_close()
      time.sleep(2.0)

    # Read the starting pose to nudge from. enable_arm() can report enabled
    # before the first joint-angle frame arrives (they are separate CAN
    # messages), so get_joint_positions can briefly raise; retry until feedback
    # is available.
    deadline = time.monotonic() + 5.0
    while True:
      try:
        target = arm.get_joint_positions()
        break
      except RuntimeError:
        if time.monotonic() >= deadline:
          raise
        time.sleep(0.05)
    target[-2] += _MOVE_DELTA

    with agilex_control.MitJointPositionController(
        arm, kp_gains=5.0, kd_gains=0.8
    ) as controller:
      print("moving ...")
      reached = controller.move_to_position(target, threshold=0.01, timeout=5.0)
      print(f"reached target: {reached}")
    # Leaving the controller context parks the arm at its rest pose and relaxes.
  finally:
    print(
        "disabling gripper and arm. WARNING: the arm will power off and drop."
    )
    try:
      # Nest so a gripper-disable failure can't skip the arm disable (leaving
      # the joint motors enabled), and disconnect always runs.
      try:
        agilex_init.disable_gripper(arm)
      finally:
        agilex_init.disable_arm(arm)
    finally:
      arm.disconnect()
    print("done. exiting.")


if __name__ == "__main__":
  main()
