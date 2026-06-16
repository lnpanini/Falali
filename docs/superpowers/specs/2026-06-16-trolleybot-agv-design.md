# TrolleyBot — Indoor Autonomous AGV Design

**Date:** 2026-06-16
**Status:** Approved design (brainstorm complete) → ready for implementation planning
**Owner:** Bryan

## 1. Goal & Scope

Build a **functional, indoor, payload-carrying autonomous guided vehicle** ("TrolleyBot") on a
Jetson Orin Nano, using **full SLAM autonomy** with integrated **computer-vision + Lidar**
navigation.

- **Environment:** indoor, flat floors, carries a real payload (a few kg).
- **Autonomy target:** build a map, localize in it, and navigate point-to-point to arbitrary goals.
- **Builder context:** new to ROS 2, willing to learn; generous (~$900+) hardware budget;
  no hardware purchased yet.
- **Out of scope (for now):** outdoor operation, multi-robot fleets, manipulation/arms,
  commercial productization.

## 2. Platform Decisions (current as of mid-2026)

| Decision | Choice | Rationale |
|---|---|---|
| Compute | Jetson Orin Nano Super Dev Kit (8GB) + NVMe SSD | Current Orin Nano; boot/work off NVMe, not SD |
| OS / SDK | **JetPack 7.x → Ubuntu 24.04** | Current native stack (JetPack 6/Ubuntu 22.04/Humble is now legacy) |
| ROS distro | **ROS 2 Jazzy (Jalisco)** | Native pairing with Ubuntu 24.04; LTS to 2029; Isaac ROS supports it on JetPack 7 |
| Autonomy stack | **Start vanilla** `slam_toolbox` + `Nav2`; **layer Isaac ROS later** | Reliable, well-documented fundamentals first; GPU-accelerated CV/VSLAM added once driving |
| Base electronics | **micro-ROS on an MCU** (Teensy 4.1 / ESP32) | MCU owns real-time motor PID + encoders + IMU and speaks ROS 2 over USB; cleanest split |

## 3. Bill of Materials (~$1,100–1,350)

| Subsystem | Part | ~Cost | Notes |
|---|---|---|---|
| Compute | Jetson Orin Nano Super Dev Kit 8GB + NVMe 500GB | $300 | |
| Lidar | **LDRobot D500** (DToF, USB) | ~$200 | `ldlidar` ROS 2 driver; pin exact branch in Phase 0 |
| Depth cam | **Orbbec Astra Pro** (structured light) | ~$150 | Indoor-only; ~0.6 m min range; `astra_camera`/OrbbecSDK ROS 2, built from source + udev; **no built-in IMU** |
| IMU | **Bosch BNO08x** (BNO080/085/086 — any; prefer 085 if buying new) | ~$20 | Mounted on the MCU via **UART-RVC mode**; gyro yaw-rate is the key signal |
| Drivetrain | 2× 12V geared motors w/ quadrature encoders + caster + payload chassis | $150 | Diff-drive |
| Motor driver | Cytron MDD10A / SmartDriveDuo | $35 | |
| MCU | Teensy 4.1 (or ESP32) | $30 | Runs micro-ROS |
| Power | 12V LiFePO4 + buck converters + fuse/switch/**e-stop** | $120 | E-stop is mandatory for a payload robot |
| Misc | Mounting plates, USB hub, wiring | $50 | |

### Sensor consequences
- **Lidar is the primary close-range obstacle sensor** (the Astra Pro is blind < 0.6 m).
- Camera adds depth coverage beyond 0.6 m + **semantic** awareness (people, etc.) — both feed Nav2 as costmap layers.
- Astra Pro is structured-light: **indoor only** (sunlight blinds it) — acceptable for this robot.
- No camera IMU → the **BNO08x provides the IMU** for `robot_localization` EKF heading correction.

## 4. System Architecture

```
┌─────────────────────── Jetson Orin Nano (ROS 2 Jazzy) ───────────────────────┐
│  Sensors→Drivers      Localization & Mapping       Navigation      Perception │
│  • ldlidar (D500) ─┐  • robot_localization (EKF) ─┐  • Nav2        • CV node   │
│  • astra_camera   ─┼─→   fuses wheel odom + IMU   ┼→   (planner,    (YOLO via  │
│  • robot_state_    │  • slam_toolbox (map/loc)   ─┘    controller,   TensorRT) │
│    publisher+URDF ─┘                                   costmaps)    → costmap  │
│                         TF tree ties it all together         ▲       layer    │
└──────────────────────────────────[ /cmd_vel ↓   /odom,/imu ↑ ]────────────────┘
                          ┌─────────────────────────────────────┐
                          │  MCU (micro-ROS): encoders, motor    │
                          │  PID, BNO08x IMU → the real-time base │
                          └─────────────────────────────────────┘
```

**CV + Lidar fusion** happens in the **Nav2 costmap**: the lidar provides the 2D geometric map and
near-field obstacle layer; the depth camera adds obstacles the lidar plane misses (low/overhanging
objects) plus a **semantic layer** (e.g. slow/stop for people). The EKF (`robot_localization`)
fuses wheel-encoder odometry with the BNO08x gyro to keep heading honest through wheel slip.

### Unit boundaries (each independently testable)
- **MCU firmware (micro-ROS):** inputs `/cmd_vel` (or wheel cmds); outputs `/odom`, `/imu`. Owns PID + encoder counting. Knows nothing about SLAM.
- **Robot description (URDF/xacro):** geometry + TF frames; drives `robot_state_publisher`; reused identically by sim and real robot.
- **Sensor drivers:** lidar + camera nodes publishing standard `sensor_msgs` on well-known topics.
- **Localization:** EKF + slam_toolbox; consumes odom/imu/scan → map + `map→odom` TF.
- **Navigation:** Nav2; consumes costmaps + goal → `/cmd_vel`.
- **Perception (CV):** consumes camera; outputs detections → costmap layer + behavior triggers.

## 5. Roadmap (each phase = its own spec → plan → build)

- **Phase 0 — Foundations** *(buildable now, no hardware):* repo + ROS 2 Jazzy workspace layout, dev environment, finalize/order BOM.
- **Phase 1 — Robot model + simulation** *(now, no hardware):* URDF/xacro of TrolleyBot, Gazebo world, drive via teleop. **Develop the whole stack in sim while parts ship.**
- **Phase 2 — Base bringup** *(hardware):* micro-ROS firmware (motors + encoder odometry + BNO08x IMU), teleop the real robot, verify odometry.
- **Phase 3 — Sensors live:** lidar (D500) + camera (Astra Pro) drivers, correct TF tree, verify in RViz.
- **Phase 4 — SLAM:** `slam_toolbox` mapping; drive around; save a map.
- **Phase 5 — Autonomous Nav2:** localize + navigate point-to-point on the saved map.
- **Phase 6 — CV layer:** object detection (TensorRT) + camera/CV costmap fusion + semantic behaviors.

**Sim-first principle:** because Nav2/SLAM are agnostic to whether encoders/scans are real or
simulated (same topics + TF), Phases 1, 4, and 5 are developed in Gazebo first, then validated on
hardware in Phases 2–3 and beyond.

## 6. Testing Strategy

- **Sim regression:** each autonomy capability (teleop, mapping, nav-to-goal) verified in Gazebo before hardware.
- **Hardware bringup checks:** odometry sanity (drive 1 m → `/odom` reads ~1 m), TF tree completeness (`tf2_tools view_frames`), sensor topic rates in RViz.
- **Safety:** hardware **e-stop** cuts motor power independent of software; software watchdog stops on lost `/cmd_vel`.

## 7. Open Items to Resolve in Planning
- Exact `ldlidar` driver branch for the D500 and `astra_camera`/OrbbecSDK build steps on JetPack 7.
- Final chassis/motor/encoder part numbers sized to the intended payload mass.
- MCU choice (Teensy 4.1 vs ESP32) and micro-ROS transport (USB-serial vs Ethernet/UDP).
