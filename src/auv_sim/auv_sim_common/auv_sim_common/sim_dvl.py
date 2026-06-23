"""Sim DVL sensor model (stand-in for the Project DAVE / Stonefish DVL plugin).

Stock Gazebo Harmonic has no DVL sensor, so this models one from sim state — exactly
what a sim DVL plugin does. It is a SENSOR MODEL, not a ground-truth shortcut: it emits
a noisy body-frame velocity with realistic bottom-lock behaviour and is the only DVL the
estimator sees. Swap it for the DAVE/Stonefish DVL by remapping topics (spec §9).

Inputs : /ground_truth/odom   nav_msgs/Odometry  (sim state; twist in body frame)
Outputs: /dvl                  auv_interfaces/DVL (frame dvl_link)
Params : noise_stddev, lock_altitude_max, dropout (failure injection), rate_hz.
"""
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from auv_interfaces.msg import DVL


class SimDVL(Node):
    def __init__(self):
        super().__init__("sim_dvl")
        self.declare_parameter("input_topic", "/ground_truth/odom")
        self.declare_parameter("frame_id", "dvl_link")
        self.declare_parameter("noise_stddev", 0.02)        # m/s
        self.declare_parameter("lock_altitude_max", 50.0)   # m, lose lock beyond
        self.declare_parameter("seafloor_z", -20.0)         # m (ENU)
        self.declare_parameter("dropout", False)            # force no-lock (failure test)
        self.declare_parameter("rate_hz", 10.0)

        self.frame = self.get_parameter("frame_id").value
        self.sigma = float(self.get_parameter("noise_stddev").value)
        self.alt_max = float(self.get_parameter("lock_altitude_max").value)
        self.floor_z = float(self.get_parameter("seafloor_z").value)
        self.dropout = bool(self.get_parameter("dropout").value)

        self._last = None
        self.create_subscription(
            Odometry, self.get_parameter("input_topic").value,
            self._on_odom, qos_profile_sensor_data)
        self.pub = self.create_publisher(DVL, "/dvl", qos_profile_sensor_data)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self._tick)
        self.get_logger().info("sim_dvl up (DAVE/Stonefish DVL stand-in)")

    def _on_odom(self, msg: Odometry):
        self._last = msg

    def _tick(self):
        if self._last is None:
            return
        msg = self._last
        z = msg.pose.pose.position.z
        altitude = max(0.0, z - self.floor_z)
        locked = (not self.dropout) and (altitude <= self.alt_max)

        v = msg.twist.twist.linear
        out = DVL()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame
        out.bottom_locked = bool(locked)
        out.altitude = float(altitude)
        if locked:
            n = np.random.normal(0.0, self.sigma, 3)
            out.velocity.x = float(v.x + n[0])
            out.velocity.y = float(v.y + n[1])
            out.velocity.z = float(v.z + n[2])
            var = self.sigma ** 2
            out.velocity_covariance = [var, 0.0, 0.0, 0.0, var, 0.0, 0.0, 0.0, var]
            out.figure_of_merit = self.sigma
            speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)
            out.beam_ranges = [altitude] * 4
            out.beam_velocities = [speed] * 4
            out.beam_valid = [True] * 4
        else:
            # No lock: invalid measurement. Estimator must gate this (spec §4, §15).
            out.figure_of_merit = float("nan")
            out.beam_valid = [False] * 4
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SimDVL()
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
