"""Keyboard teleop -> /cmd_vel. Run in its own terminal (needs stdin).

  ros2 launch auv_control teleop.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="teleop_twist_keyboard", executable="teleop_twist_keyboard",
            name="teleop_twist_keyboard", output="screen",
            prefix="xterm -e",  # give it a tty; drop if launching in a terminal
            remappings=[("/cmd_vel", "/cmd_vel")],
        ),
    ])
