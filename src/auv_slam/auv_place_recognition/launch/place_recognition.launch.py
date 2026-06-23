"""Visual + acoustic place recognition (loop-closure detection).

Runs two instances of the modality-parametric recognizer. Acoustic operates on a
mono sonar image (publish your FLS/SSS waterfall as sensor_msgs/Image, or adapt from
marine_acoustic_msgs in auv_sonar_slam).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    visual = LaunchConfiguration("visual")
    acoustic = LaunchConfiguration("acoustic")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("visual", default_value="true"),
        DeclareLaunchArgument("acoustic", default_value="false"),
        Node(package="auv_place_recognition", executable="place_recognition",
             name="place_recognition_visual", output="screen",
             condition=IfCondition(visual),
             parameters=[{"use_sim_time": use_sim_time,
                          "image_topic": "/cam/left/image_enhanced",
                          "modality": "visual"}]),
        Node(package="auv_place_recognition", executable="place_recognition",
             name="place_recognition_acoustic", output="screen",
             condition=IfCondition(acoustic),
             parameters=[{"use_sim_time": use_sim_time,
                          "image_topic": "/sonar/sss/image_mono",
                          "modality": "sss",
                          "min_time_gap": 20.0}]),
    ])
