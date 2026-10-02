"""TEMPORARY A/B check: piper_control vs agilex_control on real hardware.

Do not merge - this is a throwaway hardware-comparison script.

Streams a LIVE per-joint comparison of the two wrappers' read-only getters while
you move the arm by hand. Both wrappers stay connected at once and just listen:
CAN is a broadcast bus, so multiple read-only sockets on the same interface all
receive the same feedback frames (nothing here enables, commands, or reconfigures
the arm, so there is no bus contention). The arm is limp the whole time - you
position it by hand and watch the numbers.

Each tick it prints the agilex-minus-piper difference of:

  - joint positions (deg)
  - joint velocities (rad/s)
  - gravity torque each wrapper's position read produces through the SAME model
    (only with --model-path): both position vectors are fed to one
    GravityCompensationModel, so any per-joint tau gap is PURELY the
    position-convention difference between the SDKs.

This is the j5-float test: move j5 slowly through its range (link horizontal,
through vertical) and watch its dtau. If pyAgxArm reports j5's angle differently
from piper_sdk (sign or zero-offset), j5's dtau grows as you sweep it - that is
the wrong hover torque that makes it float on jungle but not on magenta.

Run on a machine with the arm connected (e.g. r2-shared@jungle.local):

    python compare_wrappers.py --can-port can0 --arm-type PIPER \
        --model-path <path/to/live-arm.xml>

Firmware and joint limits are printed once at startup (limits DIFFER by design:
piper_control hardcodes them, agilex reads them from the arm). Ctrl-C to stop.
"""

# pylint: disable=import-outside-toplevel,import-error

import argparse
import time

import numpy as np


def connect_piper(can_port: str, arm_type_name: str):
  """Connect via piper_control and return the interface (stays connected)."""
  from piper_control import piper_interface as pi

  return pi.PiperInterface(
      can_port, piper_arm_type=pi.PiperArmType[arm_type_name]
  )


def connect_agilex(can_port: str, arm_type_name: str):
  """Connect via agilex_control and return the interface (stays connected)."""
  from agilex_control import agilex_interface as ai

  return ai.ArmInterface(can_port, arm_type=ai.ArmType[arm_type_name])


def _print_vector(label: str, pc_vals, ac_vals) -> None:
  """Print a one-off per-joint piper-vs-agilex comparison of a vector."""
  diff = np.asarray(ac_vals) - np.asarray(pc_vals)
  print(f"{label}:")
  for i, (pc_val, ac_val) in enumerate(zip(pc_vals, ac_vals)):
    print(
        f"  J{i + 1}: piper={pc_val:+.5f}  agilex={ac_val:+.5f}  "
        f"diff={diff[i]:+.6f}"
    )
  print(f"  max|diff| = {float(np.max(np.abs(diff))):.6f}")


def main() -> None:
  """Connect both wrappers and stream the live comparison."""
  parser = argparse.ArgumentParser(description=__doc__)
  parser.add_argument("--can-port", default="can0")
  parser.add_argument(
      "--arm-type",
      default="PIPER",
      help="PIPER / PIPER_H / PIPER_X / PIPER_L",
  )
  parser.add_argument(
      "--settle",
      type=float,
      default=1.0,
      help="seconds to wait after connecting before the first read",
  )
  parser.add_argument(
      "--period",
      type=float,
      default=0.3,
      help="seconds between samples in the live loop",
  )
  parser.add_argument(
      "--model-path",
      default=None,
      help=(
          "MuJoCo XML. If given, also streams the gravity-torque diff each"
          " wrapper's joint read produces through this one model (the"
          " j5-float test)."
      ),
  )
  args = parser.parse_args()
  arm_type_name = args.arm_type.upper()

  model = None
  if args.model_path:
    from agilex_control import gravity_compensation as gc

    joint_names = [f"joint{i}" for i in range(1, 7)]
    model = gc.GravityCompensationModel(
        model_path=args.model_path, joint_names=joint_names
    )

  print("Connecting both wrappers (read-only, both listen to the CAN bus) ...")
  pc_arm = connect_piper(args.can_port, arm_type_name)
  ac_arm = connect_agilex(args.can_port, arm_type_name)
  try:
    time.sleep(args.settle)  # let feedback populate on both before reading.

    # One-off context: firmware + limits (constant, so not streamed).
    print("\n=== firmware ===")
    print(
        f"  piper={pc_arm.get_piper_firmware_version()!r}  "
        f"agilex={ac_arm.get_firmware_version()!r}"
    )
    print("\n=== joint limits (rad) - hardcoded (piper) vs arm-read (agilex) ===")
    pc_limits = pc_arm.joint_limits
    ac_limits = ac_arm.get_joint_limits()
    _print_vector("limit min", pc_limits["min"], ac_limits["min"])
    _print_vector("limit max", pc_limits["max"], ac_limits["max"])

    np.set_printoptions(precision=2, suppress=True, sign=" ")
    print(
        "\nMove the arm by hand; watch the agilex-minus-piper diffs stream."
        " Ctrl-C to stop.\n"
        "For the j5 float: sweep j5 slowly and watch dtau[4] grow if the SDKs"
        " disagree on its angle."
    )
    while True:
      try:
        pc_q = np.array(pc_arm.get_joint_positions())
        ac_q = np.array(ac_arm.get_joint_positions())
        pc_v = np.array(pc_arm.get_joint_velocities())
        ac_v = np.array(ac_arm.get_joint_velocities())
      except RuntimeError:
        # No feedback this tick from one of them; skip it.
        time.sleep(0.1)
        continue
      dq = ac_q - pc_q
      dv = ac_v - pc_v
      line = f"dq(deg): {np.degrees(dq)}  dv(rad/s): {dv}"
      if model is not None:
        dtau = np.asarray(model.predict(ac_q)) - np.asarray(
            model.predict(pc_q)
        )
        line += f"  dtau(Nm): {dtau}"
      print(line)
      time.sleep(args.period)
  except KeyboardInterrupt:
    print("\nDone.")
  finally:
    pc_arm.piper.DisconnectPort()
    ac_arm.disconnect()


if __name__ == "__main__":
  main()
