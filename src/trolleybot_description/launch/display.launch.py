"""Visualize the TrolleyBot model in RViz (no simulator needed).

Runs robot_state_publisher + joint_state_publisher_gui so wheel/lift joints can be
jogged by hand, and opens RViz with a basic config.
"""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("trolleybot_description")

    xacro_file = PathJoinSubstitution([pkg, "urdf", "trolleybot.urdf.xacro"])
    rviz_config = PathJoinSubstitution([pkg, "rviz", "trolleybot.rviz"])

    robot_description = {
        "robot_description": Command(["xacro ", xacro_file, " use_sim:=false"])
    }

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        ),
    ])
