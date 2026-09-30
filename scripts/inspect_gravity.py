"""Diagnostic: print measured joint angles vs the model's gravity torque.

Read-only -- it never enables the arm or sends torque, so the arm stays limp /
backdrivable and you move it by hand while watching the numbers. Use it to tell
whether the gravity model matches the arm (vs a joint-angle convention issue):

- At a folded / straight-down pose (links along gravity), every ``tau`` should
  be ~0. If j2/j3 are already large there, the model masses are inflated.
- Hold a link horizontal: that joint's ``tau`` should peak, and flip sign as you
  rock it through vertical. A wrong sign there = a joint fed to predict() with a
  mirrored (sign-flipped) angle.
- ``q`` sanity: near the home pose angles should read ~0, and moving a joint in
  its "+" direction should increase its angle. A surprising offset/sign = the
  angle convention differs from what the model was built against.

To run:
  python3 scripts/inspect_gravity.py --model-path <path/to/arm.xml>
"""

import argparse
import time

import numpy as np

from agilex_control import agilex_interface, gravity_compensation


def main() -> None:
  parser = argparse.ArgumentParser(
      description=__doc__,
      formatter_class=argparse.RawDescriptionHelpFormatter,
  )
  parser.add_argument(
      "--model-path", required=True, help="Path to the MuJoCo XML model."
  )
  parser.add_argument("--can-port", default="can0", help="CAN port.")
  parser.add_argument(
      "--arm-type",
      default=agilex_interface.ArmType.PIPER.name,
      choices=[arm_type.name for arm_type in agilex_interface.ArmType],
      help="Arm model.",
  )
  parser.add_argument(
      "--joint-names",
      nargs="+",
      default=None,
      help="Model joint names (default: joint1..jointN from the arm).",
  )
  args = parser.parse_args()

  arm = agilex_interface.ArmInterface(
      can_port=args.can_port,
      arm_type=agilex_interface.ArmType[args.arm_type],
  )
  print(f"Firmware: {arm.get_firmware_version()}")

  joint_names = args.joint_names or [
      f"joint{i}" for i in range(1, arm.get_num_joints() + 1)
  ]
  model = gravity_compensation.GravityCompensationModel(
      model_path=args.model_path, joint_names=joint_names
  )

  np.set_printoptions(precision=2, suppress=True, sign=" ")
  print("Move the arm by hand; watch q (deg) vs tau (Nm). Ctrl-C to stop.")
  while True:
    try:
      q = np.array(arm.get_joint_positions())
    except RuntimeError:
      time.sleep(0.1)
      continue
    tau = np.asarray(model.predict(q))
    print(f"q(deg): {np.degrees(q)}  tau(Nm): {tau}")
    time.sleep(0.3)


if __name__ == "__main__":
  main()
