"""Normalize whichever simulator's ground-truth pose onto the canonical contract.

Ground truth is for EVALUATION ONLY and must never be consumed by SLAM (spec §12).
To guarantee that, it is published on a SEPARATE frame namespace (gt_*) so it can be
overlaid in RViz without contaminating the live TF tree.

Inputs : ~input_topic (default /ground_truth/odom)  nav_msgs/Odometry  (from sim)
Outputs: /ground_truth/pose   geometry_msgs/PoseStamped  (frame gt_map)
         TF: gt_map -> gt_base_link
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class GroundTruthRepublisher(Node):
    def __init__(self):
        super().__init__("ground_truth_republisher")
        self.declare_parameter("input_topic", "/ground_truth/odom")
        self.declare_parameter("gt_world_frame", "gt_map")
        self.declare_parameter("gt_base_frame", "gt_base_link")
        self.declare_parameter("publish_tf", True)

        in_topic = self.get_parameter("input_topic").value
        self.world = self.get_parameter("gt_world_frame").value
        self.base = self.get_parameter("gt_base_frame").value
        self.publish_tf = self.get_parameter("publish_tf").value

        self.pub = self.create_publisher(PoseStamped, "/ground_truth/pose", 10)
        self.bc = TransformBroadcaster(self)
        self.sub = self.create_subscription(Odometry, in_topic, self.cb, 10)
        self.get_logger().info(
            f"ground_truth_republisher: {in_topic} -> /ground_truth/pose "
            f"(frame {self.world}); SLAM must never subscribe to this.")

    def cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header.stamp = msg.header.stamp
        ps.header.frame_id = self.world
        ps.pose = msg.pose.pose
        self.pub.publish(ps)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = self.world
            t.child_frame_id = self.base
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
            self.bc.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthRepublisher()
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
