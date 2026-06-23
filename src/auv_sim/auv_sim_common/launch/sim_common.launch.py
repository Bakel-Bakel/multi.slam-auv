"""Sim-agnostic helpers: ground-truth republisher + pressure->depth converter.

Included by both sim_gazebo.launch.py and sim_stonefish.launch.py so the two
simulators expose an identical contract (spec §7.4).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gt_input = LaunchConfiguration("ground_truth_input")
    sim_dvl = LaunchConfiguration("sim_dvl")
    sim_usbl = LaunchConfiguration("sim_usbl")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("ground_truth_input", default_value="/ground_truth/odom"),
        DeclareLaunchArgument(
            "sim_dvl", default_value="true",
            description="run the sim DVL sensor model (DAVE/Stonefish DVL stand-in)"),
        DeclareLaunchArgument(
            "sim_usbl", default_value="true",
            description="run the sim USBL sensor model"),

        Node(
            package="auv_sim_common", executable="ground_truth_republisher",
            name="ground_truth_republisher", output="screen",
            parameters=[{"use_sim_time": use_sim_time, "input_topic": gt_input}],
        ),
        Node(
            package="auv_sim_common", executable="depth_from_pressure",
            name="depth_from_pressure", output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        ),
        Node(
            package="auv_sim_common", executable="sim_dvl",
            name="sim_dvl", output="screen", condition=IfCondition(sim_dvl),
            parameters=[{"use_sim_time": use_sim_time, "input_topic": gt_input}],
        ),
        Node(
            package="auv_sim_common", executable="sim_usbl",
            name="sim_usbl", output="screen", condition=IfCondition(sim_usbl),
            parameters=[{"use_sim_time": use_sim_time, "input_topic": gt_input}],
        ),
    ])
