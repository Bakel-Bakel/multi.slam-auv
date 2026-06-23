"""Top-level orchestration (spec §13, M11).

  ros2 launch auv_bringup bringup.launch.py sim:=gazebo modalities:=all rviz:=true

Args:
  sim          : gazebo | stonefish
  modalities   : visual | sonar | all
  use_sim_time : true (default; simulation-first)
  rviz         : true|false
  record       : true|false (ros2 bag record of contract topics)

One launch arg toggles simulators (the adapters keep the contract identical), and
another toggles SLAM modalities. Nothing in the SLAM layer special-cases a simulator.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            ExecuteProcess, GroupAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _inc(pkg, rel, args, condition=None):
    src = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory(pkg), "launch", rel))
    return IncludeLaunchDescription(src, launch_arguments=args.items(),
                                    condition=condition)


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    modalities = LaunchConfiguration("modalities")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz = LaunchConfiguration("rviz")
    record = LaunchConfiguration("record")

    sim_args = {"use_sim_time": use_sim_time}
    is_gazebo = IfCondition(PythonExpression(["'", sim, "' == 'gazebo'"]))
    is_stonefish = IfCondition(PythonExpression(["'", sim, "' == 'stonefish'"]))
    visual_on = IfCondition(PythonExpression(
        ["'", modalities, "' in ('visual', 'all')"]))
    sonar_on = IfCondition(PythonExpression(
        ["'", modalities, "' in ('sonar', 'all')"]))

    rviz_cfg = PathJoinSubstitution([
        FindPackageShare("auv_bringup"), "rviz", "auv.rviz"])

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="gazebo"),
        DeclareLaunchArgument("modalities", default_value="all"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("record", default_value="false"),

        # ---- simulator (adapters normalize to the same contract) ----
        _inc("auv_sim_gazebo", "sim_gazebo.launch.py", sim_args, condition=is_gazebo),
        _inc("auv_sim_stonefish", "sim_stonefish.launch.py", sim_args,
             condition=is_stonefish),

        # ---- estimation backbone (always on) ----
        _inc("auv_state_estimation", "estimation.launch.py", sim_args),

        # ---- visual modality ----
        GroupAction([
            _inc("auv_visual_slam", "visual_slam.launch.py",
                 {"use_sim_time": use_sim_time, "backend": "rtabmap"}),
            _inc("auv_place_recognition", "place_recognition.launch.py",
                 {"use_sim_time": use_sim_time, "visual": "true", "acoustic": "false"}),
        ], condition=visual_on),

        # ---- sonar modality ----
        GroupAction([
            _inc("auv_sonar_slam", "sonar_slam.launch.py",
                 {"use_sim_time": use_sim_time, "mbes": "true", "fls": "false"}),
            _inc("auv_place_recognition", "place_recognition.launch.py",
                 {"use_sim_time": use_sim_time, "visual": "false", "acoustic": "true"}),
        ], condition=sonar_on),

        # ---- factor-graph integrator + mapping + evaluation ----
        _inc("auv_slam_core", "slam_core.launch.py", sim_args),
        _inc("auv_mapping", "mapping.launch.py", sim_args),
        _inc("auv_evaluation", "evaluate.launch.py", sim_args),

        # ---- visualization ----
        Node(package="rviz2", executable="rviz2", condition=IfCondition(rviz),
             arguments=["-d", rviz_cfg], output="screen",
             parameters=[{"use_sim_time": use_sim_time}]),

        # ---- optional recording of the contract ----
        ExecuteProcess(
            condition=IfCondition(record),
            cmd=["ros2", "bag", "record", "-o", "/tmp/auv_mission",
                 "/clock", "/imu/data", "/pressure", "/dvl", "/usbl",
                 "/cam/left/image_raw", "/cam/right/image_raw",
                 "/sonar/mbes/points", "/odom", "/slam/pose", "/slam/trajectory",
                 "/ground_truth/pose"],
            output="screen"),
    ])
