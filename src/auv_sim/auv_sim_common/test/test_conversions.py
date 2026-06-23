"""Unit tests for the coordinate/units conversions (spec §4, §12).

The depth-sign assertion here is the project's guard against the #1 catastrophic
underwater SLAM bug: feeding +d instead of z = -d into the estimator.
"""
import math

from auv_sim_common.conversions import (
    pressure_to_depth,
    depth_to_enu_z,
    pressure_to_enu_z,
    ned_to_enu_position,
    enu_to_ned_position,
    yaw_ned_to_enu,
)


def test_surface_pressure_is_zero_depth():
    assert abs(pressure_to_depth(101325.0)) < 1e-6


def test_ten_meters_depth_pressure():
    # ~1 atm per 10 m of seawater; 10 m -> ~100.5 kPa gauge.
    p = 101325.0 + 1025.0 * 9.80665 * 10.0
    assert abs(pressure_to_depth(p) - 10.0) < 1e-6


def test_depth_sign_is_negative_z_underwater():
    """The whole point: deeper => more pressure => MORE NEGATIVE z in ENU."""
    p_shallow = 101325.0 + 1025.0 * 9.80665 * 1.0    # 1 m
    p_deep = 101325.0 + 1025.0 * 9.80665 * 20.0      # 20 m
    z_shallow = pressure_to_enu_z(p_shallow)
    z_deep = pressure_to_enu_z(p_deep)
    assert z_shallow < 0.0
    assert z_deep < 0.0
    assert z_deep < z_shallow            # deeper is more negative
    assert abs(depth_to_enu_z(5.0) + 5.0) < 1e-9


def test_ned_enu_roundtrip():
    n, e, d = 3.0, -2.0, 7.0
    e2, n2, u2 = ned_to_enu_position(n, e, d)
    assert (e2, n2, u2) == (-2.0, 3.0, -7.0)
    back = enu_to_ned_position(e2, n2, u2)
    assert back == (n, e, d)


def test_yaw_ned_to_enu():
    # North (NED yaw 0) maps to ENU yaw +90 deg (pointing East-frame's North).
    assert abs(yaw_ned_to_enu(0.0) - math.pi / 2.0) < 1e-9
