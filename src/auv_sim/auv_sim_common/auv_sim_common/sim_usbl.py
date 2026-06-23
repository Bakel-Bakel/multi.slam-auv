"""Sim USBL sensor model (stand-in for the DAVE / Stonefish USBL plugin).

Emits a low-rate acoustic fix relative to a fixed surface beacon, with range/bearing/
elevation noise. The factor graph uses it as an absolute-position prior (spec §10.6).
Like sim_dvl, this is a sensor model derived from sim state, the only USBL the stack sees.

Inputs : /ground_truth/odom   nav_msgs/Odometry
Outputs: /usbl                 auv_interfaces/USBLFix (frame usbl_link)
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from auv_interfaces.msg import USBLFix


class SimUSBL(Node):
    def __init__(self):
        super().__init__("sim_usbl")
        self.declare_parameter("input_topic", "/ground_truth/odom")
        self.declare_parameter("frame_id", "usbl_link")
        self.declare_parameter("beacon_id", 1)
        self.declare_parameter("beacon_xyz", [0.0, 0.0, 0.0])  # surface beacon (ENU)
        self.declare_parameter("range_stddev", 0.3)            # m
        self.declare_parameter("angle_stddev", 0.02)           # rad
        self.declare_parameter("rate_hz", 1.0)

        self.frame = self.get_parameter("frame_id").value
        self.beacon_id = int(self.get_parameter("beacon_id").value)
        self.beacon = np.array(self.get_parameter("beacon_xyz").value, dtype=float)
        self.range_sigma = float(self.get_parameter("range_stddev").value)
        self.angle_sigma = float(self.get_parameter("angle_stddev").value)

        self._last = None
        self.create_subscription(
            Odometry, self.get_parameter("input_topic").value, self._on_odom, 10)
        self.pub = self.create_publisher(USBLFix, "/usbl", 10)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)
        self.get_logger().info("sim_usbl up (DAVE/Stonefish USBL stand-in)")

    def _on_odom(self, msg):
        self._last = msg

    def _tick(self):
        if self._last is None:
            return
        p = self._last.pose.pose.position
        rel = np.array([p.x, p.y, p.z]) - self.beacon
        rng = float(np.linalg.norm(rel)) + np.random.normal(0.0, self.range_sigma)
        bearing = math.atan2(rel[1], rel[0]) + np.random.normal(0.0, self.angle_sigma)
        horiz = math.hypot(rel[0], rel[1])
        elevation = math.atan2(rel[2], horiz) + np.random.normal(0.0, self.angle_sigma)

        out = USBLFix()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame
        out.beacon_id = self.beacon_id
        out.range = rng
        out.bearing = bearing
        out.elevation = elevation
        out.relative_position.x = float(rel[0] + np.random.normal(0.0, self.range_sigma))
        out.relative_position.y = float(rel[1] + np.random.normal(0.0, self.range_sigma))
        out.relative_position.z = float(rel[2] + np.random.normal(0.0, self.range_sigma))
        var = self.range_sigma ** 2
        out.position_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]
        out.has_absolute_fix = False
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimUSBL()
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
