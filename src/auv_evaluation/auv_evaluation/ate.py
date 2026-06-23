"""Trajectory error metrics (ATE/RPE) — pure numpy, evo-independent (spec §12).

`evo` remains the primary tool (run_evo.py wraps evo_ape/evo_rpe), but these built-ins
make accuracy assertions testable in CI without external binaries and provide the same
SE(3)-aligned ATE RMSE that evo reports with --align.
"""
import numpy as np


def umeyama_alignment(src, dst, with_scale=False):
    """Least-squares similarity transform aligning src points onto dst (Nx3).

    Returns (R, t, s). Set with_scale=False for a rigid (SE3) alignment.
    """
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    mu_s, mu_d = src.mean(0), dst.mean(0)
    sc, dc = src - mu_s, dst - mu_d
    cov = (dc.T @ sc) / len(src)
    u, d, vt = np.linalg.svd(cov)
    s = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[2, 2] = -1
    r = u @ s @ vt
    if with_scale:
        var_s = (sc ** 2).sum() / len(src)
        scale = np.trace(np.diag(d) @ s) / var_s
    else:
        scale = 1.0
    t = mu_d - scale * r @ mu_s
    return r, t, scale


def associate(est_t, est_xyz, ref_t, ref_xyz, max_dt=0.05):
    """Nearest-time association of estimate to reference samples."""
    ref_t = np.asarray(ref_t)
    pairs_e, pairs_r = [], []
    for i, t in enumerate(est_t):
        j = int(np.argmin(np.abs(ref_t - t)))
        if abs(ref_t[j] - t) <= max_dt:
            pairs_e.append(est_xyz[i])
            pairs_r.append(ref_xyz[j])
    return np.array(pairs_e), np.array(pairs_r)


def ate_rmse(est_xyz, ref_xyz, align=True, with_scale=False):
    """Absolute Trajectory Error RMSE (positions), optionally SE3/Sim3-aligned."""
    est = np.asarray(est_xyz, float)
    ref = np.asarray(ref_xyz, float)
    if len(est) < 3:
        return float("nan")
    if align:
        r, t, s = umeyama_alignment(est, ref, with_scale)
        est = (s * (r @ est.T).T) + t
    err = np.linalg.norm(est - ref, axis=1)
    return float(np.sqrt((err ** 2).mean()))


def rpe_rmse(est_xyz, ref_xyz, delta=1):
    """Relative Pose Error RMSE over translation, step `delta`."""
    est = np.asarray(est_xyz, float)
    ref = np.asarray(ref_xyz, float)
    n = min(len(est), len(ref))
    if n <= delta:
        return float("nan")
    de = est[delta:n] - est[:n - delta]
    dr = ref[delta:n] - ref[:n - delta]
    err = np.linalg.norm(de - dr, axis=1)
    return float(np.sqrt((err ** 2).mean()))


def load_tum(path):
    """Load a TUM file: 'timestamp tx ty tz qx qy qz qw'."""
    data = np.loadtxt(path)
    if data.ndim == 1:
        data = data.reshape(1, -1)
    return data[:, 0], data[:, 1:4]
