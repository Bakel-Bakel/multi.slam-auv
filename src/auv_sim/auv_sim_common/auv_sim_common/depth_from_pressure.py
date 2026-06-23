"""Convert a fluid-pressure reading into a depth/z measurement for the estimator.

THE DEPTH SIGN CONVENTION (spec §4 — the #1 catastrophic underwater SLAM bug):
  - Pressure increases as the vehicle goes deeper.
  - Depth d >= 0 downward, d = (P_abs - P_atm) / (rho * g).
  - The ROS world is ENU, where z is UP, so the surface is z = 0 and
        z = -d.
  This node publishes z = -d. Fusing +d instead silently inverts the whole
  vertical estimate. A unit test (test/test_depth_sign.py) asserts this.

Inputs : /pressure          sensor_msgs/FluidPressure   (frame pressure_link)
Outputs: /depth             geometry_msgs/PointStamped  (z = -d, frame map)
         /depth/odometry     nav_msgs/Odometry          (z position, for robot_localization)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

from auv_sim_common.conversions import pressure_to_enu_z


class DepthFromPressure(Node):
    def __init__(self):
        super().__init__("depth_from_pressure")
        self.declare_parameter("water_density", 1025.0)      # kg/m^3 (seawater)
        self.declare_parameter("gravity", 9.80665)           # m/s^2
        self.declare_parameter("atmospheric_pressure", 101325.0)  # Pa
        self.declare_parameter("pressure_units", "Pa")       # "Pa" or "kPa"
        self.declare_parameter("depth_stddev", 0.05)         # m (1-sigma)
        self.declare_parameter("output_frame", "map")

        self.rho = self.get_parameter("water_density").value
        self.g = self.get_parameter("gravity").value
        self.p_atm = self.get_parameter("atmospheric_pressure").value
        self.units = self.get_parameter("pressure_units").value
        self.depth_var = float(self.get_parameter("depth_stddev").value) ** 2
        self.frame = self.get_parameter("output_frame").value

        self.sub = self.create_subscription(
            FluidPressure, "/pressure", self.cb, qos_profile_sensor_data)
        self.pub_point = self.create_publisher(PointStamped, "/depth", 10)
        self.pub_odom = self.create_publisher(Odometry, "/depth/odometry", 10)
        self.get_logger().info(
            f"depth_from_pressure up: rho={self.rho} g={self.g} "
            f"p_atm={self.p_atm}Pa, convention z = -d")

    def pressure_to_z(self, fluid_pressure_pa: float) -> float:
        """Return ENU z (<= 0 underwater). z = -d, d = (P - P_atm)/(rho g)."""
        return pressure_to_enu_z(
            fluid_pressure_pa, atmospheric_pa=self.p_atm,
            water_density=self.rho, gravity=self.g)

    def cb(self, msg: FluidPressure):
        p = msg.fluid_pressure
        if self.units == "kPa":
            p *= 1000.0
        z = self.pressure_to_z(p)

        pt = PointStamped()
        pt.header.stamp = msg.header.stamp
        pt.header.frame_id = self.frame
        pt.point.z = z
        self.pub_point.publish(pt)

        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.frame
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.z = z
        odom.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        cov[14] = self.depth_var  # z-z entry (index 2*6+2)
        odom.pose.covariance = cov
        self.pub_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = DepthFromPressure()
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
