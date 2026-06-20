"""AprilTag detection + opennav_docking server (Phase 5 skeleton)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("trolleybot_docking")
    apriltag_cfg = os.path.join(pkg, "config", "apriltag.yaml")
    docking_cfg = os.path.join(pkg, "config", "docking.yaml")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag_node",
            output="screen",
            parameters=[apriltag_cfg, {"use_sim_time": use_sim_time}],
            remappings=[
                ("image_rect", "/camera/image"),
                ("camera_info", "/camera/camera_info"),
            ],
        ),
        Node(
            package="opennav_docking",
            executable="opennav_docking",
            name="docking_server",
            output="screen",
            parameters=[docking_cfg, {"use_sim_time": use_sim_time}],
        ),
    ])
