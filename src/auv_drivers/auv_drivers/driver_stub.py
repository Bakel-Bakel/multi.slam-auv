"""A no-op driver stub that documents the contract it must fulfil (spec §9.3).

Run with `driver:=dvl` (etc.). It logs the exact topic/type/frame a real driver must
publish, then idles. Replace the body with a real device driver to bring up hardware.
"""
import rclpy
from rclpy.node import Node

from auv_drivers.driver_contract import DRIVER_CONTRACT


class DriverStub(Node):
    def __init__(self):
        super().__init__("driver_stub")
        self.declare_parameter("driver", "dvl")
        name = self.get_parameter("driver").value
        spec = next((d for d in DRIVER_CONTRACT if d.name == name), None)
        if spec is None:
            self.get_logger().error(
                f"unknown driver '{name}'. Known: "
                f"{[d.name for d in DRIVER_CONTRACT]}")
            return
        self.get_logger().warn(
            f"[STUB] '{spec.name}' driver not implemented in v1. A real driver must "
            f"publish {spec.topic} ({spec.msg_type}) in frame '{spec.frame_id}'. "
            f"Note: {spec.notes}")


def main(args=None):
    rclpy.init(args=args)
    node = DriverStub()
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
