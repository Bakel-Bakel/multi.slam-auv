"""Compute ATE/RPE for logged trajectories (spec §12).

Tries `evo` (the spec's tool) first; falls back to the built-in numpy metrics (ate.py)
so CI works without the evo binary. Prints a table and returns a nonzero exit code if a
threshold is exceeded (used by the rosbag regression harness).
"""
import argparse
import shutil
import subprocess
import sys

from auv_evaluation.ate import load_tum, associate, ate_rmse, rpe_rmse


def run(est_tum, ref_tum, ate_threshold=None, use_evo=True):
    if use_evo and shutil.which("evo_ape"):
        try:
            out = subprocess.run(
                ["evo_ape", "tum", ref_tum, est_tum, "-a"],
                capture_output=True, text=True, check=True)
            print(out.stdout)
        except subprocess.CalledProcessError as exc:
            print(exc.stdout, exc.stderr)

    est_t, est_xyz = load_tum(est_tum)
    ref_t, ref_xyz = load_tum(ref_tum)
    e, r = associate(est_t, est_xyz, ref_t, ref_xyz, max_dt=0.2)
    if len(e) < 3:
        print("Not enough associated poses to evaluate.")
        return 2
    ate = ate_rmse(e, r, align=True)
    rpe = rpe_rmse(e, r, delta=1)
    print(f"ATE RMSE (aligned): {ate:.4f} m")
    print(f"RPE RMSE (delta=1): {rpe:.4f} m")
    if ate_threshold is not None and ate > ate_threshold:
        print(f"FAIL: ATE {ate:.4f} > threshold {ate_threshold}")
        return 1
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("estimate", help="estimate TUM file")
    ap.add_argument("reference", help="ground-truth TUM file")
    ap.add_argument("--ate-threshold", type=float, default=None)
    ap.add_argument("--no-evo", action="store_true")
    args = ap.parse_args(argv)
    sys.exit(run(args.estimate, args.reference,
                 args.ate_threshold, use_evo=not args.no_evo))


if __name__ == "__main__":
    main()
