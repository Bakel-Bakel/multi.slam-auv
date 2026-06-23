"""Modality-parametric place recognition (spec §10.5).

Detects revisits from an image stream (optical OR sonar) and emits loop-closure
candidates (auv_interfaces/LoopClosure, is_sequential=False) into auv_slam_core. Both the
visual and acoustic loop closures enter the graph through this ONE path, so the back-end
treats every modality uniformly and applies a robust kernel.

Gating to avoid false positives (spec §15.12):
  - temporal: candidate keyframe must be at least `min_time_gap` s in the past;
  - spatial:  current /odom must be within `max_revisit_distance` of the candidate
              (so we only match plausible revisits, not distant look-alikes);
  - appearance: descriptor cosine similarity must exceed `similarity_threshold`.
The relative pose seed is the odometry difference; the back-end / registration refines it.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

from auv_interfaces.msg import LoopClosure
from auv_place_recognition.descriptors import global_descriptor, cosine_similarity


class Keyframe:
    __slots__ = ("stamp", "t", "pos", "quat", "desc")

    def __init__(self, stamp, t, pos, quat, desc):
        self.stamp, self.t, self.pos, self.quat, self.desc = stamp, t, pos, quat, desc


class PlaceRecognition(Node):
    def __init__(self):
        super().__init__("place_recognition")
        self.declare_parameter("image_topic", "/cam/left/image_enhanced")
        self.declare_parameter("modality", "visual")
        self.declare_parameter("similarity_threshold", 0.85)
        self.declare_parameter("min_time_gap", 15.0)         # s
        self.declare_parameter("max_revisit_distance", 3.0)  # m
        self.declare_parameter("keyframe_distance", 0.5)     # m between stored frames

        self.modality = self.get_parameter("modality").value
        self.sim_thresh = float(self.get_parameter("similarity_threshold").value)
        self.min_gap = float(self.get_parameter("min_time_gap").value)
        self.max_dist = float(self.get_parameter("max_revisit_distance").value)
        self.kf_dist = float(self.get_parameter("keyframe_distance").value)

        self.bridge = CvBridge()
        self.kfs = []
        self._pose = None  # (pos, quat)
        self._last_kf_pos = None

        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            Image, self.get_parameter("image_topic").value,
            self._on_image, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            LoopClosure, "/slam/loop_closures", QoSProfile(depth=50))
        self.get_logger().info(
            f"place_recognition [{self.modality}] on "
            f"{self.get_parameter('image_topic').value} -> /slam/loop_closures")

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose = (np.array([p.x, p.y, p.z]),
                      np.array([q.x, q.y, q.z, q.w]))

    def _on_image(self, msg: Image):
        if self._pose is None:
            return
        try:
            enc = "bgr8" if msg.encoding in ("rgb8", "bgr8") else "mono8"
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=enc)
        except Exception:  # noqa: BLE001
            return
        pos, quat = self._pose
        if self._last_kf_pos is not None and \
                np.linalg.norm(pos - self._last_kf_pos) < self.kf_dist:
            return  # not enough motion for a new keyframe

        desc = global_descriptor(img)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        best = self._match(pos, t, desc)
        if best is not None:
            self._emit(best, msg.header.stamp, pos, quat)

        self.kfs.append(Keyframe(msg.header.stamp, t, pos, quat, desc))
        self._last_kf_pos = pos

    def _match(self, pos, t, desc):
        best, best_sim = None, self.sim_thresh
        for kf in self.kfs:
            if (t - kf.t) < self.min_gap:
                continue
            if np.linalg.norm(pos - kf.pos) > self.max_dist:
                continue
            sim = cosine_similarity(desc, kf.desc)
            if sim > best_sim:
                best, best_sim = kf, sim
        if best is not None:
            return best, best_sim
        return None

    def _emit(self, match, stamp, cur_pos, cur_quat):
        kf, sim = match
        lc = LoopClosure()
        lc.header.stamp = stamp
        lc.header.frame_id = "map"
        lc.from_stamp = kf.stamp
        lc.to_stamp = stamp
        # relative pose seed = T_from^-1 * T_to from odometry estimates
        rel = self._relative(kf.pos, kf.quat, cur_pos, cur_quat)
        lc.relative_pose.position.x = float(rel[0])
        lc.relative_pose.position.y = float(rel[1])
        lc.relative_pose.position.z = float(rel[2])
        lc.relative_pose.orientation.x = float(rel[3])
        lc.relative_pose.orientation.y = float(rel[4])
        lc.relative_pose.orientation.z = float(rel[5])
        lc.relative_pose.orientation.w = float(rel[6])
        cov = [0.0] * 36
        for i in range(3):
            cov[i * 6 + i] = 0.25      # loop seeds are loose; robust kernel + refine
        for i in range(3, 6):
            cov[i * 6 + i] = 0.1
        lc.covariance = cov
        lc.modality = self.modality
        lc.confidence = float(sim)
        lc.is_sequential = False
        self.pub.publish(lc)
        self.get_logger().info(
            f"[{self.modality}] loop candidate sim={sim:.3f} "
            f"dt={(lc.to_stamp.sec - lc.from_stamp.sec)}s")

    @staticmethod
    def _relative(p_from, q_from, p_to, q_to):
        r_from = Rotation.from_quat(q_from)
        rel_pos = r_from.inv().apply(p_to - p_from)
        rel_rot = (r_from.inv() * Rotation.from_quat(q_to)).as_quat()
        return np.concatenate([rel_pos, rel_rot])


def main(args=None):
    rclpy.init(args=args)
    node = PlaceRecognition()
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
