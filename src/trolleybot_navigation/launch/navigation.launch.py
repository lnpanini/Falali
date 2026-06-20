"""Bring up Nav2 on a saved map (Phase 4).

Usage:
    ros2 launch trolleybot_navigation navigation.launch.py map:=/path/to/map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    nav_share = get_package_share_directory("trolleybot_navigation")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")
    params_file = os.path.join(nav_share, "config", "nav2_params.yaml")

    map_yaml = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("map", description="Path to the map .yaml"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_share, "launch", "bringup_launch.py")
            ),
            launch_arguments={
                "map": map_yaml,
                "use_sim_time": use_sim_time,
                "params_file": params_file,
            }.items(),
        ),
    ])
