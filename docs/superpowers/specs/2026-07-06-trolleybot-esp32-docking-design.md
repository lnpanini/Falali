# TrolleyBot — ESP32 Under-Ride Docking Firmware Design

**Date:** 2026-07-06
**Status:** Implemented. **Rev 2026-07-07:** alignment redesigned to a 4-corner **edge + odometry**
method (see §5) after the trolley base became a guaranteed solid board; the clamp/arm mechanism is now an
external subsystem (handoff at `IClamp`).
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
| Alignment | **4× VL53L0X** ToF, one per corner (FL/FR/RL/RR) | I²C via TCA9548A mux (4 channels) |
| Odometry | dead-reckoning from commanded velocity | wheel `FG` pulse upgrade later on GPIO 40/41/42/48 |
| Safety | E-stop button | digital input |

**Kinematics:** mecanum / omni — the robot can strafe, so lateral centring is a direct sideways motion,
and orientation is a rotation in place.

**Sensing reality:** the trolley base is a **guaranteed solid board** (a board sits over the frame), so
ToF returns are **continuous and reliable**. A corner "sees the board" when its reading is valid and
within a height band; a small debounce rejects noise. (The earlier caged/intermittent assumption, and the
windowed-confidence design it required, are retired.)

**Clamp/arm subsystem is EXTERNAL:** the arm-open (until in-arm ToF pass the edge) + servo rotation +
limit-switch grip is built separately. This firmware ends at **CONFIRM (centred)** and hands off through
the `IClamp` port. In-arm sensors are future roadmap and are not implemented here.

## 3. Main Safety Principle

**The clamp must not engage unless the alignment logic CONFIRMS the platform is centred.** Limit switches
only confirm clamp *travel*; they never prove alignment. A single `SafetyMonitor` owns this gate
(`alignment_confirmed && no faults && !E-stop`); the docking state machine can only actuate the clamp
*through* it, and only reaches the handoff after a sustained `CONFIRM`.

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
   │  CornerEdgeDetector  │  IAlignmentSensor  ◄───────│  Vl53l0xMux (4 ch)     │
   │  DeadReckonOdometry  │  IOdometry (pure)          │  MecanumDrive+Bld120a  │
   │  SafetyMonitor       │  IDrive / IMotor   ◄───────│  Bts7960Clamp          │
   │  DockingStateMachine │  IClamp/ILimitSwitches◄────│  GpioLimitSwitches     │
   │                      │  IClock / ITelemetry◄──────│  ArduinoClock/Serial   │
   └──────────────────────┘                            └────────────────────────┘
              ▲ same ports
   ┌──────────┴──────────┐
   │  FAKES (host tests)  │  FakeMotor/Drive/Clamp/LimitSwitches/Odometry/Clock/Telemetry
   └─────────────────────┘
```

**Library policy:** popular libraries do all commodity work in the adapters (`pololu/VL53L0X`,
`Bounce2`, `ArduinoJson`, `SerialCommands`, ESP32 `LEDC`). Only the three things with no off-the-shelf
equivalent are hand-written — and they are the pure, unit-tested core:

- **`CornerEdgeDetector`** — debounced per-corner "board present" from the 4 ToF frames.
- **`DeadReckonOdometry`** — planar pose from integrated commanded velocity (behind `IOdometry`).
- **`SafetyMonitor`** — the single clamp-authority interlock (gated on confirmed alignment).
- **`DockingStateMachine`** — the geometric orient → centre → confirm sequence.

## 5. Domain Contracts

### CornerEdgeDetector
- Input: `AlignmentFrame` with 4 corner zones each tick — index order **FL=0, FR=1, RL=2, RR=3**.
- Per corner: `present = valid && band_min ≤ mm ≤ band_max`, with an N-sample debounce so a single spike
  cannot flip it. Solid board → no windowing needed.
- Output: `present(Corner)` booleans + raw mm.

### DeadReckonOdometry (implements `IOdometry`)
- `update(cmd, now)` integrates the commanded body velocity × dt × calibration (`max_lin_mm_s`,
  `max_ang_rad_s`) into a world-frame `Pose2D`. `reset()` zeroes.
- Robust for centring: we only ever drive to the **midpoint** between two edge events measured by the same
  integrator, so a calibration-scale error cancels. A wheel-pulse (FG) impl can replace it behind the port.

### SafetyMonitor
- `update(alignment_confirmed, faults)`; E-stop **latches** (explicit clear only, e.g. `ABORT`).
- `clampCloseAllowed() = alignment_confirmed && !motor_alarm && !clamp_overcurrent && !estop_latch`.
- `safeStopRequired() = motor_alarm || clamp_overcurrent || estop`.

### DockingStateMachine (the geometric sequence)
`IDLE → APPROACH → ORIENT → CENTER_X → CENTER_Y → CONFIRM → CLAMP_ENGAGE → CLAMPED → UNCLAMPING → IDLE`;
`FAULT` is a sticky safe state reachable anywhere. Drives `IOdometry` as it moves.
- **APPROACH:** drive forward until a front corner (FL|FR) sees the near edge.
- **ORIENT:** rotate toward the lagging corner until BOTH front corners see the board (yaw aligned).
  Purely sensor-driven — no odometry.
- **CENTER_X:** mark odom x at the near edge; drive to the far edge (front corners lose the board); drive
  to the midpoint (− front offset).
- **CENTER_Y:** strafe to find both side edges via the leading-side corner pair; drive to the midpoint.
- **CONFIRM:** all 4 corners present within tolerance, sustained past `confirm_hold_ms` → `confirmed`.
- **CLAMP_ENGAGE:** *handoff* — only if `confirmed && !safeStopRequired()`, engage the clamp (placeholder
  close to the *closed* switch). The **external** arm/servo/in-arm-ToF subsystem replaces this step.
- Any `safeStopRequired()`/E-stop → `FAULT` (drive disabled + braked, clamp stopped); `ABORT` → clear
  latch → `IDLE`; per-phase timeouts → `FAULT`.

## 6. Execution Model & Telemetry
- Cooperative fixed-rate superloop (~50 Hz control tick) driven by `IClock`; sensors read at their rate.
  The `DockingStateMachine` integrates `IOdometry` from the motion it commands (main does not).
- Telemetry: USB-CDC @115200. `STATUS` emitted as JSON (`ArduinoJson`) — `state`, `corners[4]`,
  `x_mm`, `y_mm`, `theta`, `confirmed`, plus fault flags; `DOCK/ABORT/UNCLAMP/STATUS` parsed by
  `SerialCommands`. The domain only sees the `ITelemetry` port + a `Command` enum.

## 7. Pin Budget — ESP32-S3-N16R8 (fits: ~21 of 33 usable)
Reserved on N16R8: **GPIO26–37** (flash + octal PSRAM). Usable: 0–21, 38–48. See `include/pins.h` for the
authoritative map (generated/validated with the `gpio-config` skill). Control lines are grouped for
safety: all wheel `EN` gang to one pin, all `BRK` to one pin, all `ALARM` wire-OR to one interrupt.

## 8. Testing
- **Primary (host, no hardware):** `pio test -e native` over the pure core (22 cases).
  - CornerEdgeDetector: in-band reading → present after debounce; a single out-of-band spike doesn't flip it.
  - Odometry: forward → x, strafe → y, rotate → θ.
  - Safety: `clampCloseAllowed()` denied unless `confirmed` and no faults; E-stop latches.
  - State machine: the full geometric sequence reaches `CLAMPED` only after `CONFIRM`; orient rotates
    toward the lagging corner; E-stop / over-current → `FAULT`; `ABORT` → `IDLE`.
- **Firmware compile:** `pio run -e esp32s3` (fetches `lib_deps`).
- **On hardware (later):** per-actuator jog, ToF corner map + height band, limit-switch read, odometry
  calibration, dry-run alignment with telemetry (clamp subsystem validated separately).

## 9. Open Items (for hardware bring-up)
- Confirm mecanum wheel signs, the **ORIENT rotate-direction** sign, and the centring-move signs.
- Set the ToF **height band**, the corner→mux-channel map, and centring **offsets/tolerances** in `config.h`.
- Calibrate odometry `max_lin_mm_s` / `max_ang_rad_s`; add FG wheel feedback for accuracy when needed.
- BTS7960 `IS → amps` scale and BLD120A `SV` PWM frequency, measured on the bench.
- The **external clamp/arm subsystem** (arm-open via in-arm ToF, servo rotation, limit-switch grip)
  integrates at the `IClamp` handoff.
