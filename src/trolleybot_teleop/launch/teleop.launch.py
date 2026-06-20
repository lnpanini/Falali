"""Joystick teleop -> /diff_drive_controller/cmd_vel."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory("trolleybot_teleop"), "config", "joystick.yaml"
    )

    return LaunchDescription([
        Node(package="joy", executable="joy_node", parameters=[cfg]),
        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            parameters=[cfg],
            remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel")],
        ),
    ])
