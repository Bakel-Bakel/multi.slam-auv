"""Log estimate + ground-truth trajectories to TUM files for evo / ATE (spec §12).

Records the SLAM estimate and the ground-truth pose to separate TUM-format files
('timestamp tx ty tz qx qy qz qw'). Ground truth is used for EVALUATION ONLY and is read
here, never fed back into SLAM.
"""
import os
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__("trajectory_logger")
        self.declare_parameter("output_dir", "/tmp/auv_eval")
        self.declare_parameter("estimate_topic", "/slam/pose")
        self.declare_parameter("estimate_type", "pose_cov")  # pose_cov|odom|pose
        self.declare_parameter("ground_truth_topic", "/ground_truth/pose")

        self.dir = self.get_parameter("output_dir").value
        os.makedirs(self.dir, exist_ok=True)
        self.f_est = open(os.path.join(self.dir, "estimate.tum"), "w")
        self.f_ref = open(os.path.join(self.dir, "ground_truth.tum"), "w")

        est_topic = self.get_parameter("estimate_topic").value
        est_type = self.get_parameter("estimate_type").value
        gt_topic = self.get_parameter("ground_truth_topic").value

        if est_type == "odom":
            self.create_subscription(Odometry, est_topic, self._on_odom, 20)
        elif est_type == "pose":
            self.create_subscription(PoseStamped, est_topic, self._on_pose, 20)
        else:
            self.create_subscription(
                PoseWithCovarianceStamped, est_topic, self._on_pose_cov, 20)
        self.create_subscription(PoseStamped, gt_topic, self._on_gt, 20)
        self.get_logger().info(f"trajectory_logger -> {self.dir}/*.tum")

    @staticmethod
    def _line(stamp, pose):
        t = stamp.sec + stamp.nanosec * 1e-9
        p, q = pose.position, pose.orientation
        return f"{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} " \
               f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n"

    def _on_odom(self, msg):
        self.f_est.write(self._line(msg.header.stamp, msg.pose.pose)); self.f_est.flush()

    def _on_pose(self, msg):
        self.f_est.write(self._line(msg.header.stamp, msg.pose)); self.f_est.flush()

    def _on_pose_cov(self, msg):
        self.f_est.write(self._line(msg.header.stamp, msg.pose.pose)); self.f_est.flush()

    def _on_gt(self, msg):
        self.f_ref.write(self._line(msg.header.stamp, msg.pose)); self.f_ref.flush()

    def destroy_node(self):
        try:
            self.f_est.close()
            self.f_ref.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryLogger()
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
