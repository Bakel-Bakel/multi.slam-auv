"""Convert auv_interfaces/DVL to a TwistWithCovarianceStamped for robot_localization.

Gating (spec §4, §15): if the DVL has no bottom lock the velocity is INVALID; this node
drops it so the EKF never integrates garbage (the cause of violent drift). The DVL frame
(dvl_link) is translation-only relative to base_link in the default URDF, so the velocity
vector is reused directly; if your extrinsic adds rotation, apply it here (one boundary).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistWithCovarianceStamped
from auv_interfaces.msg import DVL


class DvlToTwist(Node):
    def __init__(self):
        super().__init__("dvl_to_twist")
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("require_bottom_lock", True)
        self.frame = self.get_parameter("output_frame").value
        self.require_lock = bool(self.get_parameter("require_bottom_lock").value)

        self.sub = self.create_subscription(
            DVL, "/dvl", self._cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            TwistWithCovarianceStamped, "/dvl/twist", qos_profile_sensor_data)
        self._dropped = 0
        self.get_logger().info("dvl_to_twist up (gating invalid DVL measurements)")

    def _cb(self, msg: DVL):
        if self.require_lock and not msg.bottom_locked:
            self._dropped += 1
            if self._dropped % 20 == 1:
                self.get_logger().warn(
                    f"DVL no bottom lock: dropping measurement (count={self._dropped})")
            return

        out = TwistWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame
        out.twist.twist.linear.x = msg.velocity.x
        out.twist.twist.linear.y = msg.velocity.y
        out.twist.twist.linear.z = msg.velocity.z
        cov = [0.0] * 36
        c = msg.velocity_covariance
        # place 3x3 linear-velocity covariance into the 6x6 twist covariance block
        for i in range(3):
            for j in range(3):
                cov[i * 6 + j] = c[i * 3 + j]
        # nonzero angular variances keep the filter well-conditioned
        cov[21] = cov[28] = cov[35] = 1e6
        out.twist.covariance = cov
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = DvlToTwist()
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
