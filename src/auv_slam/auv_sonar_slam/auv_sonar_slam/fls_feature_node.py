"""Forward-looking-sonar (FLS/MSIS) feature SLAM front-end (spec §10.3).

Pipeline: sonar image -> CA-CFAR detection -> acoustic feature points -> scan-to-scan ICP
registration -> relative-pose constraints into auv_slam_core. Validate on Stonefish's
high-fidelity sonar (known ground truth) before trusting noisier DAVE/real data.

Input is taken as a mono sensor_msgs/Image (range x bearing fan). A marine_acoustic_msgs
ProjectedSonarImage adapter is a thin seam (vendor marine_acoustic_msgs, convert to the
range/bearing array, feed the same CFAR path).
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

from auv_interfaces.msg import LoopClosure
from auv_sonar_slam.cfar import ca_cfar, detections_to_points
from auv_sonar_slam.icp import icp


class FlsFeatureNode(Node):
    def __init__(self):
        super().__init__("fls_feature_slam")
        self.declare_parameter("image_topic", "/sonar/fls/image_mono")
        self.declare_parameter("modality", "fls")
        self.declare_parameter("range_max", 20.0)            # m (image bottom row)
        self.declare_parameter("fov_deg", 120.0)
        self.declare_parameter("cfar_threshold", 3.0)
        self.declare_parameter("min_features", 20)
        self.declare_parameter("icp_fitness_min", 0.5)

        self.modality = self.get_parameter("modality").value
        self.range_max = float(self.get_parameter("range_max").value)
        self.fov = np.deg2rad(float(self.get_parameter("fov_deg").value))
        self.thr = float(self.get_parameter("cfar_threshold").value)
        self.min_feat = int(self.get_parameter("min_features").value)
        self.fit_min = float(self.get_parameter("icp_fitness_min").value)

        self.bridge = CvBridge()
        self._prev_pts = None
        self._prev_stamp = None

        self.create_subscription(
            Image, self.get_parameter("image_topic").value,
            self._on_image, qos_profile_sensor_data)
        self.lc_pub = self.create_publisher(
            LoopClosure, "/slam/loop_closures", QoSProfile(depth=50))
        self.get_logger().info(
            f"fls_feature_slam [{self.modality}] up -> /slam/loop_closures")

    def extract_features(self, mono):
        mask = ca_cfar(mono, threshold_factor=self.thr)
        range_per_row = self.range_max / max(1, mono.shape[0] - 1)
        return detections_to_points(
            mask, range_per_row=range_per_row, fov_rad=self.fov)

    def _on_image(self, msg: Image):
        try:
            mono = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        except Exception:  # noqa: BLE001
            return
        pts = self.extract_features(mono.astype(np.float32))
        if len(pts) < self.min_feat:
            return

        if self._prev_pts is not None:
            r, t, fitness, err = icp(pts, self._prev_pts,
                                     max_correspondence_dist=2.0)
            if fitness >= self.fit_min:
                self._emit(r, t, fitness, msg.header.stamp)
        self._prev_pts = pts
        self._prev_stamp = msg.header.stamp

    def _emit(self, r, t, fitness, stamp):
        q = Rotation.from_matrix(r).as_quat()
        lc = LoopClosure()
        lc.header.stamp = stamp
        lc.header.frame_id = "map"
        lc.from_stamp = self._prev_stamp
        lc.to_stamp = stamp
        lc.relative_pose.position.x = float(t[0])
        lc.relative_pose.position.y = float(t[1])
        lc.relative_pose.position.z = float(t[2])
        lc.relative_pose.orientation.x = float(q[0])
        lc.relative_pose.orientation.y = float(q[1])
        lc.relative_pose.orientation.z = float(q[2])
        lc.relative_pose.orientation.w = float(q[3])
        cov = [0.0] * 36
        for i in range(3):
            cov[i * 6 + i] = 0.2
        for i in range(3, 6):
            cov[i * 6 + i] = 0.1
        lc.covariance = cov
        lc.modality = self.modality
        lc.confidence = float(fitness)
        lc.is_sequential = True
        self.lc_pub.publish(lc)


def main(args=None):
    rclpy.init(args=args)
    node = FlsFeatureNode()
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
