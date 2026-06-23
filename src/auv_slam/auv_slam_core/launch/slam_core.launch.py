"""Factor-graph integrator (M9)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("auv_slam_core")
    cfg = os.path.join(pkg, "config", "slam_core.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(package="auv_slam_core", executable="slam_core",
             name="slam_core", output="screen",
             parameters=[cfg, {"use_sim_time": use_sim_time}]),
    ])
