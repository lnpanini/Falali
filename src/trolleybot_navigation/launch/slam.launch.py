"""Run slam_toolbox in mapping mode (Phase 3).

Drive the robot around the floor, then save the map:
    ros2 run nav2_map_server map_saver_cli -f ~/trolleybot_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory("trolleybot_navigation")
    slam_params = os.path.join(nav_share, "config", "slam_toolbox_mapping.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time}],
        ),
    ])
