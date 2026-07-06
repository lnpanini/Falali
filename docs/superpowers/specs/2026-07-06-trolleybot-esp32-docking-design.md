# TrolleyBot — ESP32 Under-Ride Docking Firmware Design

**Date:** 2026-07-06
**Status:** Approved design → scaffold implemented
**Owner:** Bryan
**Supersedes direction of:** `2026-06-16-trolleybot-agv-design.md` (Jetson/ROS 2/SLAM). That document is
kept as history; this is a **deliberate pivot** to an ESP32-only prototype.

## 1. Goal & Scope

An **under-ride AMR platform** that drives beneath a textile trolley, infers from **upward-facing ToF
sensors** whether it is under the trolley and laterally centred, **clamps** onto the trolley underside,
and then moves it.

**In scope (this prototype):** ESP32-S3 firmware for sensing, motor/clamp control, safety interlocks, a
docking/clamping state machine, and serial telemetry.

**Explicitly out of scope:** LiDAR, camera vision, ROS 2, SLAM, general mapping, Raspberry Pi. The ToF
sensors are **not** for mapping — only for under-trolley alignment and clamping safety.

## 2. Hardware

| Role | Part | Control interface |
|---|---|---|
| Controller | ESP32-S3-WROOM-1 **N16R8** (octal PSRAM) | — |
| Wheel drive ×4 | **BLD120A** BLDC driver | `SV` (PWM speed), `F/R` (dir), `EN` (enable), `BRK` (brake); `FG` (speed pulse), `ALARM` (fault, active-low) |
| Clamp actuator | **BTS7960** H-bridge | `RPWM`/`LPWM` (PWM per dir), `R_EN`/`L_EN` (enable); `R_IS`/`L_IS` (analog current sense) |
| Clamp travel | 2× limit switches | open / closed |
| Alignment | N× **VL53L0X** ToF (or array part later) | I²C; multi-sensor via TCA9548A mux or XSHUT re-addressing |
| Safety | E-stop button | digital input |

**Kinematics:** mecanum / omni — the robot can strafe, so lateral centring under the trolley is a direct
sideways motion.

**Sensing reality:** the trolley underside may be caged / meshed / barred, so ToF returns are
**intermittent** — a bar one moment, a gap the next, noise/invalids often. Alignment must be built on
**confidence and repeated detection over a window**, never a single raw reading.

## 3. Main Safety Principle

**The clamp must not begin to close unless the alignment logic reports clamping is safe.** Limit switches
only confirm clamp *travel* (open/closed); they never prove the robot is correctly aligned. A single
`SafetyMonitor` owns this gate; the docking state machine can only actuate the clamp *through* it.

## 4. Architecture — Hexagonal (Ports & Adapters)

Dependency direction is strictly one-way: **domain → ports**, **adapters → ports**, **composition root
(`main.cpp`) → everything**. The domain never includes Arduino, so it compiles and unit-tests on a laptop.

```
        operator (USB serial)                     hardware
              │                                       │
        ┌─────▼─────────────────── main.cpp (composition root) ──────────────┐
        │  builds adapters, wires domain, runs the fixed-rate control loop    │
        └─────┬───────────────────────────────────────────────────┬──────────┘
              │ ports (pure interfaces)                            │ ports
   ┌──────────▼──────────┐                            ┌────────────▼───────────┐
   │  DOMAIN (pure C++)   │                            │  ADAPTERS (Arduino)    │
   │  AlignmentInterpreter│  IAlignmentSensor  ◄───────│  Vl53l0xArray / Mux    │
   │  SafetyMonitor       │  IDrive / IMotor   ◄───────│  MecanumDrive+Bld120a  │
   │  DockingStateMachine │  IClamp            ◄───────│  Bts7960Clamp          │
   │                      │  ILimitSwitches    ◄───────│  GpioLimitSwitches     │
   │                      │  IClock / ITelemetry◄──────│  ArduinoClock/Serial   │
   └──────────────────────┘                            └────────────────────────┘
              ▲ same ports
   ┌──────────┴──────────┐
   │  FAKES (host tests)  │  FakeMotor/Drive/Clamp/LimitSwitches/AlignmentSensor/Clock/Telemetry
   └─────────────────────┘
```

**Library policy:** popular libraries do all commodity work in the adapters (`pololu/VL53L0X`,
`Bounce2`, `ArduinoJson`, `SerialCommands`, ESP32 `LEDC`). Only the three things with no off-the-shelf
equivalent are hand-written — and they are the pure, unit-tested core:

- **`AlignmentInterpreter`** — turns noisy per-zone ToF frames into confidence scores.
- **`SafetyMonitor`** — the single clamp-authority interlock.
- **`DockingStateMachine`** — the docking/clamping sequence.

## 5. Domain Contracts

### AlignmentInterpreter
- Input: `AlignmentFrame { ZoneReading[{mm, valid}], zone_count, t_ms }` each tick.
- Keeps a per-zone rolling window (bitmask of recent in-band hits).
- `in-band hit` = reading valid **and** `mm` within the expected underside height band.
- Outputs `AlignmentState`:
  - `under_trolley` = mean per-zone in-band hit-rate over the window (gap-tolerant — occasional invalids
    do not collapse it).
  - `centred` = `1 − |left_rate − right_rate|`; `lateral` = signed `right_rate − left_rate` (drives strafe).
  - `fresh` = a valid reading arrived within `freshness_timeout_ms`.
  - `clamp_safe` = `under_trolley ≥ th` **and** `centred ≥ th` **and** `fresh`, sustained past
    `clamp_debounce_ms`.
- Zone geometry (which zones are left/right) is **config**, not code → sensor layout stays swappable.

### SafetyMonitor
- `clampCloseAllowed()` = `align.clamp_safe` **and** no `motor_alarm` **and** no `clamp_overcurrent`
  **and** not E-stop-latched.
- `safeStopRequired()` = `motor_alarm` **or** `clamp_overcurrent` **or** E-stop.
- E-stop **latches** — it does not auto-clear (must be explicitly cleared, e.g. by `ABORT`).

### DockingStateMachine
`IDLE → ENTERING → ALIGNING → READY_TO_CLAMP → CLAMPING → CLAMPED → UNCLAMPING → IDLE`, with `FAULT` as a
sticky safe state reachable from anywhere.
- **ENTERING:** drive forward until `under_trolley` rises (structure overhead detected).
- **ALIGNING:** strafe proportional to `lateral` until `centred` is sustained.
- **READY_TO_CLAMP:** stop; proceed only when `SafetyMonitor.clampCloseAllowed()`.
- **CLAMPING:** close the clamp until the *closed* limit switch trips; over-current or any safe-stop
  condition → `FAULT`.
- **UNCLAMPING:** open until the *open* limit switch trips.
- Any `safeStopRequired()` or E-stop at any time → `FAULT` (drive disabled + braked, clamp stopped).
- `ABORT` → safe stop + clear latch → `IDLE`.

## 6. Execution Model & Telemetry
- Cooperative fixed-rate superloop (~50 Hz control tick) driven by `IClock`; sensors read at their rate.
- Telemetry: USB-CDC @115200. `STATUS` emitted as JSON (`ArduinoJson`); `DOCK/ABORT/UNCLAMP/STATUS`
  parsed by `SerialCommands`. The domain only sees the `ITelemetry` port + a `Command` enum.

## 7. Pin Budget — ESP32-S3-N16R8 (fits: ~21 of 33 usable)
Reserved on N16R8: **GPIO26–37** (flash + octal PSRAM). Usable: 0–21, 38–48. See `include/pins.h` for the
authoritative map (generated/validated with the `gpio-config` skill). Control lines are grouped for
safety: all wheel `EN` gang to one pin, all `BRK` to one pin, all `ALARM` wire-OR to one interrupt.

## 8. Testing
- **Primary (host, no hardware):** `pio test -e native` over the pure core.
  - Interpreter: intermittent dropouts keep `under_trolley` up; sustained invalids clear `fresh`.
  - Safety: `clampCloseAllowed()` denied on low confidence / stale / motor alarm / over-current / E-stop.
  - State machine: reaches `CLAMPED` only after a granted request; `ABORT` and mid-clamp over-current
    route to a safe stop.
- **Firmware compile:** `pio run -e esp32s3` (fetches `lib_deps`).
- **On hardware (later):** per-actuator jog, ToF address self-check, limit-switch read, dry-run docking
  with the clamp motor disconnected.

## 9. Open Items (for hardware bring-up)
- Confirm mecanum wheel sign conventions and the sign of `lateral → strafe`.
- Final ToF sensor count/layout → set `zone_side[]` and the height band in `config.h`.
- BTS7960 `IS → amps` scale and BLD120A `SV` PWM frequency, measured on the bench.
