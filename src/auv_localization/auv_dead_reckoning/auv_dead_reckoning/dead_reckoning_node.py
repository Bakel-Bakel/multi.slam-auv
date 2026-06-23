"""DVL+IMU dead-reckoning reference node (spec §10.1, M6).

Publishes /dead_reckoning/odom (a baseline trajectory) WITHOUT broadcasting TF, so it
never competes with the EKF's odom->base_link edge. Used as the dead-reckoning baseline
that fused SLAM must beat in evo (spec §12).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from auv_interfaces.msg import DVL

from auv_dead_reckoning.integrator import DeadReckoner


class DeadReckoningNode(Node):
    def __init__(self):
        super().__init__("dead_reckoning")
        self.declare_parameter("output_frame", "odom")
        self.declare_parameter("child_frame", "base_link")
        self.frame = self.get_parameter("output_frame").value
        self.child = self.get_parameter("child_frame").value

        self.dr = DeadReckoner()
        self._last_t = None

        self.create_subscription(Imu, "/imu/data", self._on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(DVL, "/dvl", self._on_dvl,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(Odometry, "/dead_reckoning/odom", 10)
        self.get_logger().info("dead_reckoning up (baseline; no TF broadcast)")

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        self.dr.set_orientation(q.x, q.y, q.z, q.w)

    def _on_dvl(self, msg: DVL):
        if not msg.bottom_locked:
            return  # gate invalid measurements (drift while unlocked, as expected)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._last_t is not None:
            dt = t - self._last_t
            self.dr.integrate(
                [msg.velocity.x, msg.velocity.y, msg.velocity.z], dt)
        self._last_t = t

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.frame
        odom.child_frame_id = self.child
        odom.pose.pose.position.x = float(self.dr.position[0])
        odom.pose.pose.position.y = float(self.dr.position[1])
        odom.pose.pose.position.z = float(self.dr.position[2])
        odom.pose.pose.orientation.x = float(self.dr.orientation[0])
        odom.pose.pose.orientation.y = float(self.dr.orientation[1])
        odom.pose.pose.orientation.z = float(self.dr.orientation[2])
        odom.pose.pose.orientation.w = float(self.dr.orientation[3])
        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = DeadReckoningNode()
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
