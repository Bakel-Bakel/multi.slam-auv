"""Point-to-point ICP for sonar submap / scan registration (spec §10.3, §10.4).

Pure numpy + scipy KD-tree (no PCL dependency), so it is unit-testable and runs anywhere.
Returns the rigid transform (R, t) aligning `source` onto `target` plus a fitness score.
For production-grade bathymetric SLAM, swap in PCL GICP / probabilistic ICP at this seam;
the interface (clouds in, transform + covariance out) is unchanged.
"""
import numpy as np
from scipy.spatial import cKDTree


def _best_fit_transform(a, b):
    """Least-squares rigid transform mapping points a -> b (Kabsch/Umeyama)."""
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    aa, bb = a - ca, b - cb
    h = aa.T @ bb
    u, _, vt = np.linalg.svd(h)
    d = np.sign(np.linalg.det(vt.T @ u.T))
    s = np.diag([1.0, 1.0, d])
    r = vt.T @ s @ u.T
    t = cb - r @ ca
    return r, t


def icp(source, target, max_iterations=40, tolerance=1e-5,
        max_correspondence_dist=2.0, init_transform=None):
    """Align source onto target.

    Returns (R, t, fitness, mean_error) where fitness is the inlier ratio.
    """
    source = np.asarray(source, dtype=float)
    target = np.asarray(target, dtype=float)
    if len(source) < 3 or len(target) < 3:
        return np.eye(3), np.zeros(3), 0.0, float("inf")

    r_total = np.eye(3)
    t_total = np.zeros(3)
    src = source.copy()
    if init_transform is not None:
        r_total, t_total = init_transform
        src = (r_total @ src.T).T + t_total

    tree = cKDTree(target)
    prev_err = float("inf")
    fitness = 0.0
    mean_err = float("inf")

    for _ in range(max_iterations):
        dist, idx = tree.query(src)
        mask = dist < max_correspondence_dist
        if mask.sum() < 3:
            break
        r, t = _best_fit_transform(src[mask], target[idx[mask]])
        src = (r @ src.T).T + t
        r_total = r @ r_total
        t_total = r @ t_total + t

        mean_err = float(dist[mask].mean())
        fitness = float(mask.mean())
        if abs(prev_err - mean_err) < tolerance:
            break
        prev_err = mean_err

    return r_total, t_total, fitness, mean_err


def rotation_to_quat(r):
    """3x3 rotation -> (x,y,z,w) quaternion."""
    from scipy.spatial.transform import Rotation
    return Rotation.from_matrix(r).as_quat()
