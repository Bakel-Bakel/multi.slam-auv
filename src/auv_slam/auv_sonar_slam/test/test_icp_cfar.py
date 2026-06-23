"""Tests for ICP registration and CFAR detection."""
import numpy as np

from auv_sonar_slam.icp import icp
from auv_sonar_slam.cfar import ca_cfar, detections_to_points


def _irregular_cloud(n=80, seed=0):
    # Sparse, irregular terrain-like cloud (no grid aliasing for NN correspondences).
    rng = np.random.default_rng(seed)
    xy = rng.uniform(-5.0, 5.0, size=(n, 2))
    z = 0.8 * np.sin(0.6 * xy[:, 0]) + 0.5 * np.cos(0.4 * xy[:, 1])
    return np.column_stack([xy, z])


def test_icp_recovers_known_translation():
    target = _irregular_cloud()
    shift = np.array([0.4, -0.3, 0.1])
    source = target + shift
    r, t, fitness, err = icp(source, target, max_correspondence_dist=3.0)
    recovered = (r @ source.T).T + t
    assert err < 0.02
    assert fitness > 0.95
    assert np.max(np.abs(recovered - target)) < 0.02


def test_icp_recovers_known_yaw():
    target = _irregular_cloud(seed=3)
    th = np.deg2rad(8.0)
    rot = np.array([[np.cos(th), -np.sin(th), 0],
                    [np.sin(th), np.cos(th), 0],
                    [0, 0, 1]])
    source = (rot @ target.T).T
    r, t, fitness, err = icp(source, target, max_correspondence_dist=3.0)
    recovered = (r @ source.T).T + t
    assert err < 0.02
    assert np.max(np.abs(recovered - target)) < 0.05


def test_cfar_detects_bright_target():
    img = np.full((60, 60), 10.0, dtype=np.float32)
    img[30, 30] = 250.0
    mask = ca_cfar(img, guard=2, train=6, threshold_factor=3.0)
    assert mask[30, 30]
    assert mask.sum() < 50          # mostly background, not a flood of detections


def test_detections_to_points_geometry():
    mask = np.zeros((100, 51), dtype=bool)
    mask[50, 25] = True             # mid-range, center bearing -> straight ahead
    pts = detections_to_points(mask, range_per_row=0.1, fov_rad=2.0)
    assert pts.shape == (1, 3)
    assert abs(pts[0, 1]) < 1e-6    # center bearing -> y ~ 0
    assert pts[0, 0] > 0            # forward
