"""Preset: visual modality only. `sim:=gazebo|stonefish`."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    rviz = LaunchConfiguration("rviz")
    bringup = os.path.join(
        get_package_share_directory("auv_bringup"), "launch", "bringup.launch.py")
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="gazebo"),
        DeclareLaunchArgument("rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup),
            launch_arguments={"sim": sim, "modalities": "visual",
                              "rviz": rviz}.items()),
    ])
