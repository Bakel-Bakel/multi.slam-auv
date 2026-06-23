"""Thruster allocation node (body twist -> per-thruster forces)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = PathJoinSubstitution([
        FindPackageShare("auv_control"), "config", "allocation.yaml"])
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="auv_control", executable="thruster_allocator",
            name="thruster_allocator", output="screen",
            parameters=[config, {"use_sim_time": use_sim_time}],
        ),
    ])
