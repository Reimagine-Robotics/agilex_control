"""Example of using the MIT position controller to move the arm a small amount.

To run this example:
python3 scripts/simple_move.py
"""

import time

from agilex_control import agilex_control, agilex_interface, can_utils

# How far to nudge the 2nd-to-last joint, in radians.
_MOVE_DELTA = 0.2


def main() -> None:
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

  arm = agilex_interface.ArmInterface(can_port=ports[0])
  try:
    arm.set_installation_pos(agilex_interface.ArmInstallationPos.UPRIGHT)

    # Enable the motors, retrying until they report enabled. The robust enable
    # loop will move to agilex_init once it is ported (mirroring piper_init).
    print("enabling arm")
    deadline = time.monotonic() + 5.0
    while not arm.enable_arm():
      if time.monotonic() >= deadline:
        raise TimeoutError("Timed out enabling the arm.")
      time.sleep(0.1)

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
    print("disabling arm. WARNING: it will power off and may drop.")
    try:
      arm.disable_arm()
    finally:
      arm.disconnect()
    print("done. exiting.")


if __name__ == "__main__":
  main()
