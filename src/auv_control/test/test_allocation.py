"""Tests for the thrust-allocation matrix construction."""
import numpy as np

from auv_control.thruster_allocator import build_tam, rotation_from_rpy


VECTORED6 = [
    [0.156, 0.111, 0.0, 0.0, 0.0, -0.7853981],
    [0.156, -0.111, 0.0, 0.0, 0.0, 0.7853981],
    [-0.156, 0.111, 0.0, 0.0, 0.0, 0.7853981],
    [-0.156, -0.111, 0.0, 0.0, 0.0, -0.7853981],
    [0.120, 0.218, 0.0, 0.0, 1.5707963, 0.0],
    [-0.120, 0.218, 0.0, 0.0, 1.5707963, 0.0],
]


def test_rotation_identity():
    assert np.allclose(rotation_from_rpy(0, 0, 0), np.eye(3))


def test_vertical_thruster_pushes_z():
    # pitch +90 deg maps body +x to -z (downward thrust direction).
    d = rotation_from_rpy(0.0, 1.5707963, 0.0) @ np.array([1.0, 0.0, 0.0])
    assert abs(d[0]) < 1e-6 and abs(d[1]) < 1e-6
    assert abs(abs(d[2]) - 1.0) < 1e-6


def test_tam_shape_and_rank():
    tam = build_tam(VECTORED6)
    assert tam.shape == (6, 6)
    # Horizontal thrusters give surge/sway/yaw; verticals give heave/roll/pitch.
    assert np.linalg.matrix_rank(tam) >= 5


def test_surge_command_is_achievable():
    tam = build_tam(VECTORED6)
    thrust = np.linalg.pinv(tam) @ np.array([10.0, 0, 0, 0, 0, 0])
    wrench = tam @ thrust
    assert wrench[0] > 0.0            # produces +x force
    assert abs(wrench[1]) < 1e-6      # no sway
