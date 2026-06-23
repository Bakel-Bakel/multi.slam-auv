"""Body-velocity command -> per-thruster forces for the BlueROV2.

The thrust-allocation matrix (TAM) is built from the thruster geometry so it stays
consistent with urdf/thrusters.xacro. Column i of the TAM is the wrench produced by a
unit force on thruster i:  [ d_i ; p_i x d_i ]  where d_i is the thrust direction
(R(rpy_i) * x_hat) and p_i its position in body frame. Thrust = pinv(TAM) @ wrench.

Inputs : /cmd_vel               geometry_msgs/Twist     (desired body twist)
Outputs: /<prefix>/<name>        std_msgs/Float64        (thrust [N] per thruster,
                                                          bridged to the gz Thruster)
"""
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64


def rotation_from_rpy(roll, pitch, yaw):
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def build_tam(poses):
    """poses: list of [x,y,z,roll,pitch,yaw]; returns 6xN allocation matrix."""
    cols = []
    x_hat = np.array([1.0, 0.0, 0.0])
    for x, y, z, roll, pitch, yaw in poses:
        d = rotation_from_rpy(roll, pitch, yaw) @ x_hat
        p = np.array([x, y, z])
        col = np.concatenate([d, np.cross(p, d)])
        cols.append(col)
    return np.column_stack(cols)


class ThrusterAllocator(Node):
    def __init__(self):
        super().__init__("thruster_allocator")
        self.declare_parameter("thruster_topic_prefix", "bluerov2")
        self.declare_parameter("thruster_names",
                               ["thruster1", "thruster2", "thruster3",
                                "thruster4", "thruster5", "thruster6"])
        self.declare_parameter("thruster_poses_flat", [0.0])  # 6*N flat, from yaml
        self.declare_parameter("wrench_gains", [40.0, 40.0, 40.0, 8.0, 8.0, 12.0])
        self.declare_parameter("max_thrust", 40.0)
        self.declare_parameter("timeout_sec", 1.0)
        self.declare_parameter("rate_hz", 50.0)

        prefix = self.get_parameter("thruster_topic_prefix").value
        self.names = list(self.get_parameter("thruster_names").value)
        flat = list(self.get_parameter("thruster_poses_flat").value)
        self.gains = np.array(self.get_parameter("wrench_gains").value, dtype=float)
        self.max_thrust = float(self.get_parameter("max_thrust").value)
        self.timeout = float(self.get_parameter("timeout_sec").value)
        rate = float(self.get_parameter("rate_hz").value)

        # thruster_poses arrives flattened as one list of 6*N floats from YAML lists.
        poses = self._reshape_poses(flat, len(self.names))
        self.tam = build_tam(poses)
        self.tam_pinv = np.linalg.pinv(self.tam)
        self.get_logger().info(
            f"TAM {self.tam.shape}, cond={np.linalg.cond(self.tam):.2f}, "
            f"{len(self.names)} thrusters")

        self.pubs = [self.create_publisher(Float64, f"/{prefix}/{n}", 10)
                     for n in self.names]
        self.last_cmd = np.zeros(6)
        self.last_stamp = self.get_clock().now()
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)
        self.create_timer(1.0 / rate, self.tick)

    @staticmethod
    def _reshape_poses(flat, n):
        # YAML list-of-lists is delivered flattened; if it is a clean 6N, reshape.
        if len(flat) == 6 * n:
            return [flat[6 * i:6 * i + 6] for i in range(n)]
        # Fallback: assume already list-of-rows (rare param typing path).
        return flat

    def on_cmd(self, msg: Twist):
        self.last_cmd = np.array([msg.linear.x, msg.linear.y, msg.linear.z,
                                  msg.angular.x, msg.angular.y, msg.angular.z])
        self.last_stamp = self.get_clock().now()

    def tick(self):
        dt = (self.get_clock().now() - self.last_stamp).nanoseconds * 1e-9
        cmd = self.last_cmd if dt < self.timeout else np.zeros(6)
        wrench = self.gains * cmd
        thrusts = self.tam_pinv @ wrench
        thrusts = np.clip(thrusts, -self.max_thrust, self.max_thrust)
        for pub, t in zip(self.pubs, thrusts):
            pub.publish(Float64(data=float(t)))


def main(args=None):
    rclpy.init(args=args)
    node = ThrusterAllocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
