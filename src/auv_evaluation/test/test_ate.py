"""Tests for the ATE/RPE metrics."""
import numpy as np

from auv_evaluation.ate import (umeyama_alignment, ate_rmse, rpe_rmse, associate)


def _traj(n=50):
    t = np.linspace(0, 10, n)
    xyz = np.column_stack([np.cos(t), np.sin(t), -0.1 * t])
    return t, xyz


def test_identical_trajectories_zero_ate():
    _, xyz = _traj()
    assert ate_rmse(xyz, xyz, align=True) < 1e-6


def test_ate_invariant_to_rigid_transform():
    _, xyz = _traj()
    th = 0.7
    rot = np.array([[np.cos(th), -np.sin(th), 0],
                    [np.sin(th), np.cos(th), 0], [0, 0, 1]])
    moved = (rot @ xyz.T).T + np.array([3.0, -2.0, 1.0])
    # ATE with alignment should be ~0 since it's the same trajectory transformed.
    assert ate_rmse(moved, xyz, align=True) < 1e-6


def test_umeyama_recovers_transform():
    _, xyz = _traj()
    r, t, s = umeyama_alignment(xyz, xyz + np.array([1, 2, 3]))
    assert np.allclose(t, [1, 2, 3], atol=1e-6)
    assert abs(s - 1.0) < 1e-6


def test_ate_detects_drift():
    t, xyz = _traj()
    drift = xyz + np.column_stack([0.0 * t, 0.0 * t, 0.05 * t])  # growing z drift
    assert ate_rmse(drift, xyz, align=True) > 0.01


def test_associate_matches_nearest():
    t = np.array([0.0, 1.0, 2.0])
    xyz = np.zeros((3, 3))
    e, r = associate(t, xyz, t + 0.01, xyz, max_dt=0.05)
    assert len(e) == 3
