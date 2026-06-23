"""Pure DVL+IMU dead-reckoning integrator (no ROS) so it is trivially unit-tested.

Body-frame velocity (DVL) is rotated into the world frame by the current IMU orientation
and integrated. This is the well-understood baseline the fused estimate is compared to.
"""
import numpy as np


def quat_to_rotation_matrix(x, y, z, w):
    """Unit quaternion (x,y,z,w) -> 3x3 rotation (body->world)."""
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


class DeadReckoner:
    """Accumulates world-frame position from body velocities + orientations."""

    def __init__(self, position=None):
        self.position = np.zeros(3) if position is None else np.asarray(position, float)
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # x,y,z,w

    def set_orientation(self, x, y, z, w):
        self.orientation = np.array([x, y, z, w], dtype=float)

    def integrate(self, body_velocity, dt):
        """Advance the position by body_velocity (m/s, body frame) over dt seconds."""
        if dt <= 0.0:
            return self.position
        rot = quat_to_rotation_matrix(*self.orientation)
        world_vel = rot @ np.asarray(body_velocity, dtype=float)
        self.position = self.position + world_vel * dt
        return self.position
