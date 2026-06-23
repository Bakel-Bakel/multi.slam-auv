"""Underwater image enhancement node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    method = LaunchConfiguration("method")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("method", default_value="clahe_wb",
                              description="none|wb|clahe|clahe_wb|full"),
        Node(package="auv_image_enhancement", executable="image_enhancer",
             name="image_enhancer", output="screen",
             parameters=[{"use_sim_time": use_sim_time, "method": method}]),
    ])
