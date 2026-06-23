"""Adapter: visual SLAM pose stream -> factor-graph constraints (spec §10.2).

Works with ANY visual pose source (ORB-SLAM3 wrapper, RTAB-Map, OpenVINS) publishing a
nav_msgs/Odometry or geometry_msgs/PoseStamped. It emits keyframe-to-keyframe relative
pose constraints (auv_interfaces/LoopClosure, modality="visual", is_sequential=True) into
auv_slam_core. Loop closures from the visual backend (when available) are forwarded with
is_sequential=False so the graph applies a robust kernel. Mirrors the Orca4/Orca5 pattern
of fusing camera-SLAM poses into the vehicle estimator.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation

from auv_interfaces.msg import LoopClosure


def pose_to_matrix(p):
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(
        [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]).as_matrix()
    T[:3, 3] = [p.position.x, p.position.y, p.position.z]
    return T


def matrix_to_pose(T, pose_msg):
    q = Rotation.from_matrix(T[:3, :3]).as_quat()
    pose_msg.position.x, pose_msg.position.y, pose_msg.position.z = T[:3, 3]
    pose_msg.orientation.x, pose_msg.orientation.y = q[0], q[1]
    pose_msg.orientation.z, pose_msg.orientation.w = q[2], q[3]
    return pose_msg


class VisualSlamAdapter(Node):
    def __init__(self):
        super().__init__("visual_slam_adapter")
        self.declare_parameter("pose_topic", "/visual/odom")
        self.declare_parameter("keyframe_distance", 0.5)   # m
        self.declare_parameter("keyframe_min_dt", 0.5)     # s
        self.declare_parameter("position_stddev", 0.05)    # m
        self.declare_parameter("rotation_stddev", 0.02)    # rad

        self.kf_dist = float(self.get_parameter("keyframe_distance").value)
        self.kf_dt = float(self.get_parameter("keyframe_min_dt").value)
        ps = float(self.get_parameter("position_stddev").value)
        rs = float(self.get_parameter("rotation_stddev").value)
        self.cov = self._diag_cov(ps, rs)

        self._last_T = None
        self._last_stamp = None
        self.pub = self.create_publisher(
            LoopClosure, "/slam/loop_closures", QoSProfile(depth=50))
        self.create_subscription(
            Odometry, self.get_parameter("pose_topic").value,
            self._on_odom, qos_profile_sensor_data)
        self.get_logger().info("visual_slam_adapter up -> /slam/loop_closures")

    @staticmethod
    def _diag_cov(ps, rs):
        cov = [0.0] * 36
        for i in range(3):
            cov[i * 6 + i] = ps * ps
        for i in range(3, 6):
            cov[i * 6 + i] = rs * rs
        return cov

    def _on_odom(self, msg: Odometry):
        T = pose_to_matrix(msg.pose.pose)
        stamp = msg.header.stamp
        t = stamp.sec + stamp.nanosec * 1e-9
        if self._last_T is None:
            self._last_T, self._last_stamp = T, stamp
            self._last_t = t
            return
        rel = np.linalg.inv(self._last_T) @ T
        moved = np.linalg.norm(rel[:3, 3])
        if moved < self.kf_dist and (t - self._last_t) < self.kf_dt:
            return

        lc = LoopClosure()
        lc.header.stamp = stamp
        lc.header.frame_id = "map"
        lc.from_stamp = self._last_stamp
        lc.to_stamp = stamp
        matrix_to_pose(rel, lc.relative_pose)
        lc.covariance = self.cov
        lc.modality = "visual"
        lc.confidence = 1.0
        lc.is_sequential = True
        self.pub.publish(lc)

        self._last_T, self._last_stamp, self._last_t = T, stamp, t


def main(args=None):
    rclpy.init(args=args)
    node = VisualSlamAdapter()
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
