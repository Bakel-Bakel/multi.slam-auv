"""Pure coordinate/units conversions for the AUV stack (spec §4).

Kept dependency-free and side-effect-free so they are trivially unit-tested
(test/test_conversions.py). Convert at exactly ONE boundary; never inside SLAM.
"""
import math


def pressure_to_depth(pressure_pa: float,
                      atmospheric_pa: float = 101325.0,
                      water_density: float = 1025.0,
                      gravity: float = 9.80665) -> float:
    """Hydrostatic depth d >= 0 (downward) from absolute fluid pressure."""
    return (pressure_pa - atmospheric_pa) / (water_density * gravity)


def depth_to_enu_z(depth: float) -> float:
    """ENU z from downward depth. Pressure rises with depth, ENU z is up: z = -d."""
    return -depth


def pressure_to_enu_z(pressure_pa: float, **kw) -> float:
    """Convenience: absolute pressure straight to ENU z (<= 0 underwater)."""
    return depth_to_enu_z(pressure_to_depth(pressure_pa, **kw))


def ned_to_enu_position(n: float, e: float, d: float):
    """NED (North-East-Down) position -> ENU (East-North-Up)."""
    return (e, n, -d)


def enu_to_ned_position(e: float, n: float, u: float):
    """ENU -> NED."""
    return (n, e, -u)


def yaw_ned_to_enu(yaw_ned: float) -> float:
    """Heading: NED yaw (CW from North) -> ENU yaw (CCW from East)."""
    return (math.pi / 2.0) - yaw_ned
