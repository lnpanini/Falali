# TrolleyBot

A factory-floor **Autonomous Mobile Robot (AMR)** that navigates by **2D LIDAR** and docks
under trolleys using **fiducial-marker computer vision**. Built on **ROS 2 Jazzy** with a
**simulation-first** workflow (Gazebo Harmonic), targeting a **CPU-only** compute board.

## Architecture at a glance

| Job | Approach | Key packages |
|-----|----------|--------------|
| Safety obstacle-stop | Hardware safety-rated LIDAR / e-stop relay (independent of compute) | — |
| Localization & mapping | 2D LIDAR SLAM | `slam_toolbox` |
| Navigation & avoidance | Costmap + planners on a pre-built map | `nav2` |
| (Optional) 3D obstacle layer | Depth cam / 3D LIDAR voxel layer for off-plane obstacles | `spatio_temporal_voxel_layer` |
| Docking onto trolleys | AprilTag/ArUco + approach controller + lift/latch | `apriltag_ros`, `opennav_docking` |
| Odometry | Wheel encoders + IMU fusion | `robot_localization` |
| Base control | Diff-drive | `ros2_control`, `diff_drive_controller` |

> **Design note:** No VLA / neural model is used in the navigation, obstacle-avoidance, or docking
> control loops. See `docs/` / the design assessment for why classical LIDAR + fiducial CV beats a
> vision-language model on a fixed factory floor.

## Language strategy

- **C++** — `ros2_control` hardware interface and any Nav2/docking plugins (mandatory pluginlib + perf).
- **Python (rclpy)** — launch files, params, bringup, mission orchestration, glue.
- **Rust** — microcontroller firmware (Embassy/RTIC), below ROS. Optional isolated rclrs nodes.

## Workspace layout (`src/`)

- `trolleybot_description` — URDF/xacro robot model, `ros2_control` macro, display launch.
- `trolleybot_simulation` — Gazebo Harmonic factory world, tagged trolley models, sim bringup + bridge.
- `trolleybot_hardware` — custom `ros2_control` `SystemInterface` for the DIY diff-drive base.
- `trolleybot_navigation` — slam_toolbox, Nav2 params, localization, maps.
- `trolleybot_docking` — AprilTag detection + `opennav_docking` config + lift/latch action server.
- `trolleybot_orchestrator` — mission state machine (pickup → dock → lift → dropoff → lower → undock).
- `trolleybot_msgs` — custom actions/messages.
- `trolleybot_bringup` — top-level launch for the real robot.
- `trolleybot_teleop` — joystick/keyboard teleop.

## Build

Requires **ROS 2 Jazzy** on **Ubuntu 24.04** and **Gazebo Harmonic**.

```bash
# from the workspace root
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Try it (Phase 1 — drives in sim)

```bash
# visualize the robot model in RViz
ros2 launch trolleybot_description display.launch.py

# spawn the robot in the Gazebo factory world and drive it
ros2 launch trolleybot_simulation sim.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_drive_controller/cmd_vel
```

## Roadmap

0. Scaffolding (this commit) · 1. Description + sim · 2. DIY hardware interface (HW gate) ·
3. SLAM · 4. Localization + Nav2 · 4.5 Optional 3D obstacle layer · 5. Docking ·
6. Mission orchestration · 7. Field hardening.
