"""State-estimation backbone: DVL->twist gating + robot_localization EKF (M6).

Publishes smooth /odom and the odom->base_link TF. map->odom is owned by auv_slam_core.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("auv_state_estimation")
    ekf_cfg = os.path.join(pkg, "config", "ekf.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),

        Node(
            package="auv_state_estimation", executable="dvl_to_twist",
            name="dvl_to_twist", output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="robot_localization", executable="ekf_node",
            name="ekf_filter_node", output="screen",
            parameters=[ekf_cfg, {"use_sim_time": use_sim_time}],
        ),
    ])
