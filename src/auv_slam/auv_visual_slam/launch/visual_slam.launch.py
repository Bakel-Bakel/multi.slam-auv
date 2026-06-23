"""Visual SLAM front-end (M7): image enhancement + a visual backend + the adapter.

backend:=rtabmap  -> stereo RTAB-Map (binary ROS 2 package, easy cross-check)
backend:=orbslam3 -> vendored ros2_orb_slam3 (see README seam); falls back to adapter-only
backend:=none     -> adapter only (expects an external /visual/odom)
"""
import os
from ament_index_python.packages import (get_package_share_directory,
                                          PackageNotFoundError)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    backend = LaunchConfiguration("backend")
    enh = get_package_share_directory("auv_image_enhancement")

    actions = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("backend", default_value="rtabmap",
                              description="rtabmap|orbslam3|none"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(enh, "launch", "image_enhancement.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items()),
        Node(package="auv_visual_slam", executable="visual_slam_adapter",
             name="visual_slam_adapter", output="screen",
             parameters=[{"use_sim_time": use_sim_time}]),
    ]

    is_rtabmap = PythonExpression(["'", backend, "' == 'rtabmap'"])
    try:
        rtab = get_package_share_directory("rtabmap_launch")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rtab, "launch", "rtabmap.launch.py")),
            condition=IfCondition(is_rtabmap),
            launch_arguments={
                "stereo": "true",
                "left_image_topic": "/cam/left/image_enhanced",
                "right_image_topic": "/cam/right/image_enhanced",
                "left_camera_info_topic": "/cam/left/camera_info",
                "right_camera_info_topic": "/cam/right/camera_info",
                "frame_id": "base_link",
                "odom_topic": "/visual/odom",
                "use_sim_time": use_sim_time,
                "rviz": "false", "rtabmap_viz": "false",
            }.items()))
    except PackageNotFoundError:
        actions.append(LogInfo(
            condition=IfCondition(is_rtabmap),
            msg="rtabmap_launch not found; install ros-humble-rtabmap-ros."))

    return LaunchDescription(actions)
