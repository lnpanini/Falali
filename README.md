# TrolleyBot — ESP32 Under-Ride Docking Firmware

Firmware for an AMR-style **under-ride platform** that drives beneath a textile trolley, uses
upward-facing time-of-flight sensors to confirm it is under the trolley and centred, **clamps** onto
the underside, and then moves the trolley.

- **Controller:** ESP32-S3-WROOM-1 **N16R8** (16 MB flash, 8 MB octal PSRAM)
- **No** LiDAR, camera, ROS 2, SLAM, mapping, or Raspberry Pi — the ESP32 is the entire control stack.

Full design: [`docs/superpowers/specs/2026-07-06-trolleybot-esp32-docking-design.md`](docs/superpowers/specs/2026-07-06-trolleybot-esp32-docking-design.md).

## Safety principle

**The clamp must not begin to close unless the alignment logic reports clamping is safe.** Limit
switches only confirm clamp *travel*; they never authorise clamping. `SafetyMonitor` is the single
gate, and the docking state machine can only actuate the clamp through it.

## Architecture — hexagonal (ports & adapters)

```
lib/ports      pure C++ interfaces + shared types (no Arduino)
lib/domain     the hand-written core:  AlignmentInterpreter · SafetyMonitor · DockingStateMachine
lib/drive      MecanumDrive (mixes body-frame vx/vy/omega -> 4 wheels)
lib/fakes      desktop test doubles for every port
lib/hal_esp32  Arduino adapters over popular libraries
include/       config.h (tunables) · pins.h (GPIO map)
src/main.cpp   composition root: builds adapters, wires domain, runs the loop
test/          host unit tests (Unity)
```

Dependency rule: **domain → ports** only; **adapters → ports**; **main → everything**. The domain
never includes Arduino, so it compiles and tests on a laptop. Commodity work uses popular libraries
(`pololu/VL53L0X`, `Bounce2`, `ArduinoJson`, `SerialCommands`, ESP32 `LEDC`); only the docking,
alignment-confidence, and safety logic is hand-written — and that is the part under test.

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
{"state":"ALIGNING","under":0.94,"centred":0.71,"lateral":0.29,"fresh":true,
 "clamp_safe":false,"motor_alarm":false,"overcurrent":false,"estop":false}
```

## Pin map — ESP32-S3-N16R8

Authoritative map: [`include/pins.h`](include/pins.h) (validated with the `gpio-config` skill: 21 pins,
0 errors). Reserved on the N16R8: **GPIO26–37** (flash + octal PSRAM) and **GPIO19/20** (USB-CDC).

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

1. **Arduino-ESP32 core ≥ 3.0** (the LEDC PWM API used here). Older cores need `ledcSetup`/`ledcAttachPin`.
2. **BLD120A polarities** — `F/R` forward sense, and active-low `EN`/`BRK`/`ALARM` (`Bld120aMotor.h`).
3. **Mecanum sign conventions** — confirm forward / strafe / yaw directions (`MecanumDrive.cpp`), and the
   sign of `lateral → strafe` in `DockingStateMachine` ALIGNING.
4. **BTS7960 current scale** — calibrate `amps_per_volt` and `cfg::kClampStallAmps`.
5. **ToF layout** — set `cfg::kNumZones`, the mux channels, `zone_side[]`, and the `band_*_mm` height
   band for your actual sensor placement. Default backend is the TCA9548A mux (`Vl53l0xMux`);
   `Vl53l0xArray` (XSHUT re-addressing) is a drop-in alternative.
6. **Dry-run docking** with the clamp motor disconnected and watch the JSON telemetry.
