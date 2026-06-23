"""Bring up the BlueROV2 in Stonefish, publishing into the SAME contract as Gazebo (M4).

Requires the vendored Stonefish library + `stonefish_ros2` (scripts/build_stonefish.sh).
The Stonefish-side adapter is: (a) sensor topic names chosen in the scenario, (b) the
remaps below, and (c) a NED->ENU normalization for marine-convention outputs handled in
the sim-common adapters. Downstream SLAM packages are untouched (spec §7.4).
"""
import os
from ament_index_python.packages import (get_package_share_directory,
                                          PackageNotFoundError)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _try_share(pkg):
    try:
        return get_package_share_directory(pkg)
    except PackageNotFoundError:
        return None


def generate_launch_description():
    pkg = get_package_share_directory("auv_sim_stonefish")
    pkg_desc = get_package_share_directory("auv_description")
    pkg_common = get_package_share_directory("auv_sim_common")
    pkg_control = get_package_share_directory("auv_control")

    use_sim_time = LaunchConfiguration("use_sim_time")
    scenario = LaunchConfiguration("scenario")
    rate = LaunchConfiguration("rate")
    resolution = LaunchConfiguration("resolution")

    actions = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "scenario",
            default_value=os.path.join(pkg, "scenarios", "bluerov2.scn")),
        DeclareLaunchArgument("rate", default_value="300.0"),
        DeclareLaunchArgument("resolution", default_value="1200 800"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_desc, "launch", "description.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time,
                              "use_gazebo": "false"}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_common, "launch", "sim_common.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_control, "launch", "control.launch.py")),
            launch_arguments={"use_sim_time": use_sim_time}.items()),
    ]

    sf = _try_share("stonefish_ros2")
    if sf is not None:
        actions.append(Node(
            package="stonefish_ros2", executable="stonefish_simulator",
            name="stonefish", output="screen",
            arguments=[scenario, rate, resolution],
            parameters=[{"use_sim_time": use_sim_time}],
        ))
    else:
        actions.append(LogInfo(msg=(
            "stonefish_ros2 not found. Build it first: "
            "src/auv_sim/auv_sim_stonefish/scripts/build_stonefish.sh . "
            "The rest of the contract (description/common/control) is up.")))

    return LaunchDescription(actions)
