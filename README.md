# Falali — a 30.007 project

ESP32-S3 under-ride docking firmware for **TrolleyBot** — an AMR-style **under-ride platform** that
drives beneath a textile trolley, uses upward-facing time-of-flight sensors to confirm it is under
the trolley and centred, **clamps** onto the underside, and then moves the trolley.

- **Controller:** ESP32-S3-WROOM-1 **N16R8** (16 MB flash, 8 MB octal PSRAM)
- **No** LiDAR, camera, ROS 2, SLAM, or mapping.

> **Architecture change in progress (2026-08-04).** With 4× AS5600 wheel encoders and a BNO085 IMU
> added, control moves to a **Raspberry Pi 5** driving two ESP32s over USB serial (base + arm). The
> ESP32 is no longer the entire control stack. See
> [`docs/superpowers/specs/2026-08-04-rpi5-main-controller-design.md`](docs/superpowers/specs/2026-08-04-rpi5-main-controller-design.md)
> and the [Pi setup runbook](docs/rpi5-setup.md). Everything below still describes the current,
> working single-ESP firmware — migrate in the order the spec gives.
>
> **Hardware documentation lives in [`docs/hardware/`](docs/hardware/)** — power-up runbook,
> component list, per-connector pinouts for all three PCBs (extracted from the KiCad netlists, not
> inferred), and a fabrication/repair guide. [`docs/hardware-architecture.md`](docs/hardware-architecture.md)
> is the older companion and is partly superseded by it. The pin table further down this README is
> the *pre-PCB design intent* and does **not** match the fabricated board.

Full design: [`docs/superpowers/specs/2026-07-06-trolleybot-esp32-docking-design.md`](docs/superpowers/specs/2026-07-06-trolleybot-esp32-docking-design.md).

## Safety principle

**The clamp must not engage unless the alignment logic CONFIRMS the platform is centred.** Limit
switches only confirm clamp *travel*; they never authorise clamping. `SafetyMonitor` is the single
gate (`confirmed && no faults && !E-stop`), and the docking state machine reaches the clamp handoff
only after a sustained `CONFIRM`. The clamp/arm mechanism itself is an **external subsystem** that
plugs in at the `IClamp` port.

## Architecture — hexagonal (ports & adapters)

```
lib/ports      pure C++ interfaces + shared types (no Arduino)
lib/domain     the hand-written core:  CornerEdgeDetector · DeadReckonOdometry · SafetyMonitor · DockingStateMachine
lib/drive      MecanumDrive (mixes body-frame vx/vy/omega -> 4 wheels)
lib/fakes      desktop test doubles for every port
lib/hal_esp32  Arduino adapters over popular libraries
include/       config.h (tunables) · pins.h (GPIO map)
src/main.cpp   composition root: builds adapters, wires domain, runs the loop
test/          host unit tests (Unity)
```

Dependency rule: **domain → ports** only; **adapters → ports**; **main → everything**. The domain
never includes Arduino, so it compiles and tests on a laptop. Commodity work uses popular libraries
(`pololu/VL53L0X`, `Bounce2`, `ArduinoJson`, `SerialCommands`, ESP32 `LEDC`); only the docking
sequence, corner edge detection, odometry, and safety logic are hand-written — and that is the part
under test.

**Alignment method:** with a solid trolley board and 4 corner ToF sensors, docking is a deterministic
geometric sequence — `APPROACH → ORIENT` (rotate until both front corners see the edge) `→ CENTER_X`
(odometry-centre between near/far edges) `→ CENTER_Y` (same, strafing) `→ CONFIRM → clamp handoff`.
Centring uses dead-reckoning odometry; driving to the midpoint between two edge events makes it robust
to calibration error.

## Build, test, flash

```bash
# Host unit tests of the pure core — no hardware needed:
pio test -e native

# Compile / upload the firmware (fetches lib_deps):
pio run -e esp32s3
pio run -e esp32s3 -t upload
pio device monitor -b 115200
```

## Serial interface (USB-CDC @115200)

Commands in (newline-terminated): `DOCK`, `ABORT`, `UNCLAMP`, `STATUS`.
Status out is JSON, e.g.:

```json
{"state":"CENTER_X","corners":[true,true,false,false],"x_mm":142.0,"y_mm":-3.0,
 "theta":0.02,"confirmed":false,"motor_alarm":false,"overcurrent":false,"estop":false}
```
(`corners` is `[FL, FR, RL, RR]`.)

## Pin map — ESP32-S3-N16R8

> ## 🛑 The table below is WRONG. Do not wire or flash from it.
>
> It is the **pre-PCB design intent** and does not match the fabricated board. On the real board
> GPIO4–7 are the **ToF XSHUT** lines, not wheel SV, and GPIO38/39 are **motor brake** lines, not
> I²C. Using this map would drive the I²C bus as motor enables.
>
> **The real map is [`include/pins.h`](include/pins.h)**, extracted from the KiCad netlist, with
> per-connector pinouts in [`docs/hardware/boards.md`](docs/hardware/boards.md).
>
> Kept here only because it is referenced by older notes. It should be deleted.

Reserved on the N16R8: **GPIO26–37** (flash + octal PSRAM) and **GPIO19/20** (USB-CDC).

| Group | Signals | GPIO |
|---|---|---|
| Wheel speed (SV, PWM) | FL/FR/RL/RR | 4, 5, 6, 7 |
| Wheel direction (F/R) | FL/FR/RL/RR | 15, 16, 17, 18 |
| Wheel enable / brake (ganged) | EN / BRK | 8 / 9 |
| Wheel fault (wire-OR) | ALARM | 10 |
| Clamp (BTS7960) | RPWM / LPWM / EN | 11 / 12 / 13 |
| Clamp current sense (ADC1) | IS close / open | 1 / 2 |
| Limit switches | open / closed | 14 / 21 |
| E-stop | button | 47 |
| ToF I²C | SDA / SCL | 38 / 39 |

## Hardware bring-up checklist (before trusting it under a trolley)

The `hal_esp32` adapters compile against the libraries but are **not yet hardware-validated**. On the
bench, verify and adjust:

1. **PWM API** — a `PwmPin` shim supports Arduino-ESP32 core 2.x *and* 3.x automatically.
2. **BLD120A polarities** — `F/R` forward sense, and active-low `EN`/`BRK`/`ALARM` (`Bld120aMotor.h`).
3. **Mecanum + orient signs** — confirm forward / strafe / yaw directions (`MecanumDrive.cpp`) and the
   `ORIENT` rotate-direction sign in `DockingStateMachine.cpp` (rotate toward the lagging corner).
4. **ToF corners** — mount FL/FR/RL/RR to mux channels `{0,1,2,3}` (`cfg::kMuxChannels`); set the board
   **height band** (`makeCornerConfig`). Default backend is `Vl53l0xMux`; `Vl53l0xArray` (XSHUT) is a
   drop-in alternative.
5. **Odometry calibration** — measure `max_lin_mm_s` / `max_ang_rad_s` (`makeOdometryCal`); tune the
   centring tolerance/offsets in `makeDockConfig`.
6. **BTS7960 current scale** — calibrate `amps_per_volt` and `cfg::kClampStallAmps`.
7. **Dry-run the alignment** and watch the JSON telemetry (`corners`, `x_mm`, `y_mm`, `confirmed`). The
   clamp/arm subsystem is external and validated separately at the `IClamp` handoff.
