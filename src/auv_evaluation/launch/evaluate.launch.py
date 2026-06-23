"""Trajectory logging for evo evaluation (M10)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    estimate_topic = LaunchConfiguration("estimate_topic")
    estimate_type = LaunchConfiguration("estimate_type")
    output_dir = LaunchConfiguration("output_dir")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("estimate_topic", default_value="/slam/pose"),
        DeclareLaunchArgument("estimate_type", default_value="pose_cov",
                              description="pose_cov|odom|pose"),
        DeclareLaunchArgument("output_dir", default_value="/tmp/auv_eval"),
        Node(package="auv_evaluation", executable="trajectory_logger",
             name="trajectory_logger", output="screen",
             parameters=[{"use_sim_time": use_sim_time,
                          "estimate_topic": estimate_topic,
                          "estimate_type": estimate_type,
                          "output_dir": output_dir}]),
    ])
