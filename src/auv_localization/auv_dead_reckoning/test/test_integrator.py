"""Tests for the dead-reckoning integrator."""
import math
import numpy as np

from auv_dead_reckoning.integrator import DeadReckoner, quat_to_rotation_matrix


def test_identity_quaternion_is_identity():
    assert np.allclose(quat_to_rotation_matrix(0, 0, 0, 1), np.eye(3))


def test_straight_line_forward():
    dr = DeadReckoner()
    # 1 m/s forward for 10 x 1 s steps -> 10 m along world x.
    for _ in range(10):
        dr.integrate([1.0, 0.0, 0.0], 1.0)
    assert abs(dr.position[0] - 10.0) < 1e-9
    assert abs(dr.position[1]) < 1e-9


def test_yaw_90_moves_along_world_y():
    dr = DeadReckoner()
    # yaw +90 deg about z: body +x maps to world +y.
    half = math.pi / 4.0
    dr.set_orientation(0.0, 0.0, math.sin(half), math.cos(half))
    dr.integrate([2.0, 0.0, 0.0], 3.0)
    assert abs(dr.position[0]) < 1e-9
    assert abs(dr.position[1] - 6.0) < 1e-6


def test_zero_dt_no_move():
    dr = DeadReckoner()
    dr.integrate([5.0, 5.0, 5.0], 0.0)
    assert np.allclose(dr.position, np.zeros(3))
