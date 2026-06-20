"""Top-level bringup for the REAL TrolleyBot base (Phase 2+).

Loads the robot description with the DIY hardware interface (use_sim:=false),
starts the ros2_control controller manager + controllers, and the EKF.
LIDAR/IMU drivers are left as TODOs until the sensors are chosen.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    desc_share = get_package_share_directory("trolleybot_description")
    nav_share = get_package_share_directory("trolleybot_navigation")

    xacro_file = os.path.join(desc_share, "urdf", "trolleybot.urdf.xacro")
    controllers = os.path.join(desc_share, "config", "controllers.yaml")
    ekf_cfg = os.path.join(nav_share, "config", "ekf.yaml")

    robot_description = {
        "robot_description": Command(["xacro ", xacro_file, " use_sim:=false"])
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers],
    )

    jsb_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    diff_drive_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["diff_drive_controller"],
    )
    lift_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["lift_position_controller"],
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_cfg],
    )

    return LaunchDescription([
        robot_state_publisher,
        controller_manager,
        ekf,
        RegisterEventHandler(
            OnProcessExit(target_action=jsb_spawner,
                          on_exit=[diff_drive_spawner, lift_spawner])
        ),
        jsb_spawner,
        # TODO(phase2): add the LIDAR driver and IMU driver nodes here.
    ])
