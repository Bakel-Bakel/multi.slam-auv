"""Bring up the BlueROV2 in Gazebo Harmonic with underwater physics + bridge (M3).

Launches: gz-sim (ocean world) -> spawn robot from /robot_description -> ros_gz_bridge
(the Gazebo-side adapter) -> robot_state_publisher -> sim-common helpers -> thruster
allocation. Everything lands on the interface-contract topics/frames so downstream
SLAM is simulator-agnostic.

Harmonic note (Humble): apt ros_gz uses ignition-transport11 (Fortress). Harmonic 8
uses gz-transport13. When gz_version:=8 (default), spawn goes through the native
spawn_harmonic helper; build ros_gz for Harmonic once via
scripts/build_ros_gz_harmonic.sh so the bridge can talk to gz sim 8 too.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_gazebo = get_package_share_directory("auv_sim_gazebo")
    pkg_desc = get_package_share_directory("auv_description")
    pkg_common = get_package_share_directory("auv_sim_common")
    pkg_control = get_package_share_directory("auv_control")
    ros_gz_sim = get_package_share_directory("ros_gz_sim")

    use_sim_time = LaunchConfiguration("use_sim_time")
    heavy = LaunchConfiguration("heavy")
    world = LaunchConfiguration("world")
    spawn_z = LaunchConfiguration("spawn_z")
    headless = LaunchConfiguration("headless")
    gz_version = LaunchConfiguration("gz_version")
    spawn_delay = LaunchConfiguration("spawn_delay")

    world_path = PathJoinSubstitution([pkg_gazebo, "worlds", world])
    bridge_cfg = os.path.join(pkg_gazebo, "config", "bridge.yaml")

    is_harmonic = PythonExpression(["'", gz_version, "' == '8'"])

    gz_args_headless = [world_path, " -s -r -v3"]
    gz_args_gui = [world_path, " -r -v3"]

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        condition=IfCondition(headless),
        launch_arguments={
            "gz_args": gz_args_headless,
            "gz_version": gz_version,
        }.items(),
    )
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")),
        condition=UnlessCondition(headless),
        launch_arguments={
            "gz_args": gz_args_gui,
            "gz_version": gz_version,
        }.items(),
    )

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, "launch", "description.launch.py")),
        launch_arguments={
            "use_sim_time": use_sim_time, "heavy": heavy, "use_gazebo": "true",
        }.items(),
    )

    # Fortress/Garden path: stock ros_gz create (transport matches gz sim 6/7).
    spawn_fortress = Node(
        package="ros_gz_sim", executable="create", output="screen",
        condition=UnlessCondition(is_harmonic),
        arguments=["-world", world, "-topic", "robot_description", "-name", "bluerov2",
                   "-z", spawn_z, "-allow_renaming", "true"],
    )

    # Harmonic path: native gz service spawn (apt ros_gz create uses transport 11).
    spawn_harmonic = Node(
        package="auv_sim_gazebo", executable="spawn_harmonic.py", output="screen",
        condition=IfCondition(is_harmonic),
        parameters=[{
            "world": world,
            "entity_name": "bluerov2",
            "spawn_z": spawn_z,
            "param_node": "robot_state_publisher",
            "wait_timeout": 120.0,
        }],
    )

    delayed_spawn = TimerAction(period=spawn_delay, actions=[spawn_fortress, spawn_harmonic])

    bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        parameters=[{"config_file": bridge_cfg, "use_sim_time": use_sim_time}],
    )

    sim_common = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_common, "launch", "sim_common.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, "launch", "control.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("heavy", default_value="false"),
        DeclareLaunchArgument("world", default_value="ocean.sdf"),
        DeclareLaunchArgument("spawn_z", default_value="-1.0"),
        DeclareLaunchArgument(
            "headless", default_value="false",
            description="Run gz-sim server only (-s). Use when no display/GPU GUI."),
        DeclareLaunchArgument(
            "gz_version", default_value="8",
            description="Gazebo Sim major version (Harmonic = 8). Must match `gz sim --version`."),
        DeclareLaunchArgument(
            "spawn_delay", default_value="8.0",
            description="Seconds to wait before spawning (Gazebo GUI can be slow to start)."),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                               os.path.join(pkg_gazebo, "worlds")),
        gz_sim_headless, gz_sim_gui, description, delayed_spawn, bridge, sim_common, control,
    ])
