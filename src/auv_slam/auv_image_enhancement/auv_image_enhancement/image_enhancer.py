"""Underwater image enhancement node (spec §10.2).

Subscribes raw camera streams and republishes an enhanced stream that visual SLAM
consumes. Classical methods (CLAHE + gray-world WB + UDCP dehaze) are built in; a learned
model (FUnIE-GAN / Sea-thru) can be slotted behind the same `method` parameter later.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from auv_image_enhancement.enhance import enhance


class ImageEnhancer(Node):
    def __init__(self):
        super().__init__("image_enhancer")
        self.declare_parameter("method", "clahe_wb")
        self.declare_parameter("cameras", ["left", "right"])
        self.method = self.get_parameter("method").value
        cams = list(self.get_parameter("cameras").value)

        self.bridge = CvBridge()
        self.pubs = {}
        for cam in cams:
            in_topic = f"/cam/{cam}/image_raw"
            out_topic = f"/cam/{cam}/image_enhanced"
            self.pubs[cam] = self.create_publisher(
                Image, out_topic, qos_profile_sensor_data)
            self.create_subscription(
                Image, in_topic,
                lambda msg, c=cam: self._cb(msg, c), qos_profile_sensor_data)
            self.get_logger().info(f"{in_topic} -> {out_topic} [{self.method}]")

    def _cb(self, msg: Image, cam: str):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            out = enhance(bgr, self.method)
            out_msg = self.bridge.cv2_to_imgmsg(out, encoding="bgr8")
            out_msg.header = msg.header
            self.pubs[cam].publish(out_msg)
        except Exception as exc:  # noqa: BLE001 - keep node alive on bad frames
            self.get_logger().warn(f"enhance failed on {cam}: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageEnhancer()
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
