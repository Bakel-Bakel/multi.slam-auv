"""Sonar SLAM front-ends (M8): MBES submap SLAM + FLS/MSIS feature SLAM."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    mbes = LaunchConfiguration("mbes")
    fls = LaunchConfiguration("fls")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("mbes", default_value="true"),
        DeclareLaunchArgument("fls", default_value="false"),
        Node(package="auv_sonar_slam", executable="mbes_submap_slam",
             name="mbes_submap_slam", output="screen",
             condition=IfCondition(mbes),
             parameters=[{"use_sim_time": use_sim_time}]),
        Node(package="auv_sonar_slam", executable="fls_feature_slam",
             name="fls_feature_slam", output="screen",
             condition=IfCondition(fls),
             parameters=[{"use_sim_time": use_sim_time}]),
    ])
