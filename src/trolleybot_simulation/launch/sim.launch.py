"""Bring up TrolleyBot in the Gazebo Harmonic factory world.

Starts Gazebo, publishes the robot description, spawns the robot, bridges the
Gazebo sensor/clock topics to ROS, and loads the ros2_control controllers.

Drive it with:
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\
        --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("trolleybot_simulation")
    desc_share = get_package_share_directory("trolleybot_description")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(sim_share, "worlds", "factory.sdf"),
        description="Absolute path to the SDF world file.",
    )

    xacro_file = os.path.join(desc_share, "urdf", "trolleybot.urdf.xacro")
    robot_description = {
        "robot_description": Command(["xacro ", xacro_file, " use_sim:=true"]),
        "use_sim_time": True,
    }

    bridge_config = os.path.join(sim_share, "config", "gz_bridge.yaml")

    # Let Gazebo resolve `model://trolley` from this package's models directory.
    resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.path.join(sim_share, "models")
        + os.pathsep
        + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": [world, " -r -v 4"]}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", "robot_description", "-name", "trolleybot", "-z", "0.15"],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
    )

    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )
    lift_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lift_position_controller"],
    )

    # Load controllers only after the robot (and its gz_ros2_control manager) exists.
    controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[jsb_spawner])
    )
    drive_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[diff_drive_spawner, lift_spawner])
    )

    return LaunchDescription([
        world_arg,
        resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        bridge,
        controllers_after_spawn,
        drive_after_jsb,
    ])
