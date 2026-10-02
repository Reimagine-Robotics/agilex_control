"""Inspect a sysid sample .npz for bad poses (no hardware needed).

Two failure modes from hand collection:
  - You released DURING the 0.7s record -> the torque steps mid-window ->
    high within-pose std (efforts_std).
  - You kept supporting the arm the WHOLE time -> steady but under-read, since
    your hand bore the load -> measured torque well BELOW the model's prediction
    on the loaded joints.

Prints a per-pose table and flags outliers on both. With --model-path it also
compares each pose's measured torque to the model's gravity prediction (use the
same model you'll fit from; an imperfect prior still exposes gross under-reads).

  python3 scripts/inspect_sysid_samples.py --samples-path samples_scarlet.npz \
      [--model-path /tmp/scarlet_model/mjmodel.xml]
"""

import argparse

import numpy as np

from agilex_control import gravity_compensation


def main() -> None:
  parser = argparse.ArgumentParser(
      description=__doc__,
      formatter_class=argparse.RawDescriptionHelpFormatter,
  )
  parser.add_argument("--samples-path", required=True, help="sysid .npz.")
  parser.add_argument(
      "--model-path",
      default=None,
      help="Optional MuJoCo XML to compare measured vs predicted torque.",
  )
  parser.add_argument(
      "--std-factor",
      type=float,
      default=3.0,
      help="Flag a pose if its max effort-std exceeds this * median.",
  )
  args = parser.parse_args()

  data = np.load(args.samples_path)
  qpos, eff, std = data["qpos"], data["efforts"], data["efforts_std"]
  num_poses, num_joints = eff.shape
  np.set_printoptions(precision=2, suppress=True, sign=" ")
  print(f"{num_poses} poses, {num_joints} joints\n")

  model_tau = None
  if args.model_path is not None:
    joint_names = [f"joint{i}" for i in range(1, num_joints + 1)]
    model = gravity_compensation.GravityCompensationModel(
        model_path=args.model_path, joint_names=joint_names
    )
    model_tau = np.array([np.asarray(model.predict(q)) for q in qpos])

  print("Per-joint measured effort across poses (Nm) -- a tiny span means that")
  print("joint is never loaded, so it can't be identified:")
  for j in range(num_joints):
    col = eff[:, j]
    print(
        f"  j{j + 1}: min={col.min():6.2f}  mean={col.mean():6.2f}"
        f"  max={col.max():6.2f}  span={col.max() - col.min():5.2f}"
    )
  print()

  if model_tau is not None:
    resid = eff - model_tau
    print("Per-joint residual (measured - model) across poses, N.m -- point")
    print("--model-path at the FITTED model to see how well it matched each")
    print(
        "joint. A large rms/mean here means the fit did NOT match that joint:"
    )
    for j in range(num_joints):
      col = resid[:, j]
      rms = float(np.sqrt(np.mean(col**2)))
      print(
          f"  j{j + 1}: rms={rms:5.2f}  max|r|={np.abs(col).max():5.2f}"
          f"  mean={col.mean():+5.2f}"
      )
    print()

  std_max = std.max(axis=1)
  std_med = float(np.median(std_max))
  std_thresh = args.std_factor * max(std_med, 1e-6)
  print(
      f"median per-pose max-std = {std_med:.3f} Nm; flag > {std_thresh:.3f}\n"
  )

  for i in range(num_poses):
    flags = []
    if std_max[i] > std_thresh:
      flags.append("UNSTABLE(std)")
    line = (
        f"pose {i:2d}  |eff|max={np.abs(eff[i]).max():5.2f}  "
        f"std_max={std_max[i]:5.2f}"
    )
    if model_tau is not None:
      resid = eff[i] - model_tau[i]
      # Under-read: measured notably smaller in magnitude than predicted on the
      # joint the model loads most (hand still bearing load).
      j = int(np.argmax(np.abs(model_tau[i])))
      pred, meas = model_tau[i][j], eff[i][j]
      ratio = meas / pred if abs(pred) > 0.2 else float("nan")
      line += (
          f"  |resid|max={np.abs(resid).max():5.2f}"
          f"  j{j + 1} meas/pred={ratio:5.2f}"
      )
      if np.isfinite(ratio) and ratio < 0.5:
        flags.append(f"UNDER-READ(j{j+1})")
    if flags:
      line += "   <-- " + ", ".join(flags)
    print(line)

  print("\nUNSTABLE(std): likely released mid-record -> re-capture that pose.")
  if model_tau is not None:
    print(
        "UNDER-READ: measured << predicted on the loaded joint -> you may have"
        " been supporting it (or the model prior is just off there)."
    )
    print("qpos(deg) of flagged poses, for reference:")
    for i in range(num_poses):
      bad = std_max[i] > std_thresh
      if model_tau is not None:
        j = int(np.argmax(np.abs(model_tau[i])))
        pred = model_tau[i][j]
        if abs(pred) > 0.2 and eff[i][j] / pred < 0.5:
          bad = True
      if bad:
        print(f"  pose {i:2d}: {np.degrees(qpos[i])}")


if __name__ == "__main__":
  main()
