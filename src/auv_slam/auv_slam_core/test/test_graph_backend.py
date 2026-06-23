"""Tests for the GTSAM pose-graph backend: a drifted loop should close."""
import numpy as np
import gtsam

from auv_slam_core.graph_backend import PoseGraphBackend, pose3


def _rel(dx, dy, dyaw):
    half = dyaw / 2.0
    return pose3([dx, dy, 0.0], [0.0, 0.0, np.sin(half), np.cos(half)])


def test_loop_closure_reduces_drift():
    g = PoseGraphBackend(prior_sigma=0.01)
    g.add_first_keyframe(pose3([0, 0, 0], [0, 0, 0, 1]))

    # Square loop: 4 sides of 2 m with 90-deg turns, but inject yaw drift each step.
    sigmas = [0.05] * 6
    prev = 0
    drift = np.deg2rad(8.0)
    for _ in range(4):
        prev = g.add_odometry_keyframe(prev, _rel(2.0, 0.0, np.pi / 2 + drift), sigmas)

    est_before = g.update()
    last = g.latest_index()
    # A perfect square returns to the start (kf4 == kf0). With yaw drift the optimized
    # end pose lands away from the start.
    start = est_before[0].translation()
    end_before = est_before[last].translation()
    drift_dist = np.linalg.norm(end_before - start)
    assert drift_dist > 0.1

    # Physically the vehicle is back at the start: loop closure (last -> 0) = identity.
    identity = pose3([0, 0, 0], [0, 0, 0, 1])
    g.add_loop_closure(last, 0, identity, [0.02] * 6, robust=True)
    est_after = g.update(extra_iterations=10)

    # After closing, the last keyframe must coincide with the first.
    rel = est_after[last].between(est_after[0])
    residual = np.linalg.norm(gtsam.Pose3.Logmap(rel))
    assert residual < drift_dist          # closure pulled them together
    assert residual < 0.1


def test_position_prior_pulls_estimate():
    g = PoseGraphBackend(prior_sigma=1.0)
    g.add_first_keyframe(pose3([0, 0, 0], [0, 0, 0, 1]))
    idx = g.add_odometry_keyframe(0, _rel(1.0, 0.0, 0.0), [0.5] * 6)
    # USBL says the second keyframe is actually at (5, 0, -3).
    g.add_position_prior(idx, [5.0, 0.0, -3.0], 0.05)
    est = g.update(extra_iterations=5)
    p = est[idx].translation()
    assert abs(p[0] - 5.0) < 0.3
    assert abs(p[2] + 3.0) < 0.3
