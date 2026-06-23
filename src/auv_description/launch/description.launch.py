"""Bring up robot_state_publisher (and optionally RViz) for the BlueROV2.

This publishes the full static TF tree from the URDF (spec M2). It is included by
every higher-level launch file; it is the only publisher of base_link->sensor_* edges.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("auv_description")
    xacro_file = PathJoinSubstitution([pkg, "urdf", "bluerov2.urdf.xacro"])

    use_sim_time = LaunchConfiguration("use_sim_time")
    heavy = LaunchConfiguration("heavy")
    use_gazebo = LaunchConfiguration("use_gazebo")
    rviz = LaunchConfiguration("rviz")
    gui = LaunchConfiguration("gui")

    robot_description = Command([
        "xacro ", xacro_file,
        " heavy:=", heavy,
        " use_gazebo:=", use_gazebo,
    ])

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("heavy", default_value="false"),
        DeclareLaunchArgument("use_gazebo", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("gui", default_value="false",
                              description="joint_state_publisher_gui for standalone viewing"),

        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "robot_description": ParameterValue(robot_description, value_type=str),
            }],
        ),
        Node(
            package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
            condition=IfCondition(gui), output="screen",
        ),
        Node(
            package="rviz2", executable="rviz2", condition=IfCondition(rviz),
            arguments=["-d", PathJoinSubstitution([pkg, "config", "description.rviz"])],
            parameters=[{"use_sim_time": use_sim_time}], output="screen",
        ),
    ])
