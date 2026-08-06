# Raspberry Pi 5 as main controller — design

**Date:** 2026-08-04
**Status:** design, not yet implemented
**Supersedes:** the "no Raspberry Pi — the ESP32 is the entire control stack" claim in `README.md`

## Why this changes

Four AS5600 wheel encoders and a BNO085 IMU arrive with a problem the single-ESP stack was never
built for: **state estimation**. Today `DeadReckonOdometry` integrates the *command* — it assumes the
robot did what it was told. With encoders and an IMU there is now real measurement to fuse, wheel
slip becomes observable, and the fusion maths (and its tuning) wants a filesystem, floating-point
headroom, log replay, and a language you can iterate in. That is the Pi's job.

The second ESP is a subsystem boundary, not a capacity problem. The arm/clamp was *already* an
external subsystem in this codebase — `IClamp` is a port precisely so the mechanism could be swapped
or moved. Putting it on its own microcontroller is that port becoming a physical wire.

## Topology

```
                    ┌──────────────────────────────┐
                    │  Raspberry Pi 5   (the brain) │
                    │                               │
                    │  state estimation (enc + IMU) │
                    │  docking sequencer            │
                    │  mecanum mixing               │
                    │  logging / replay / operator  │
                    └───┬───────────────────────┬───┘
                 USB-CDC│ 115200/921600         │USB-CDC
                        │                       │
            ┌───────────▼──────────┐  ┌─────────▼───────────┐
            │  ESP-BASE  (S3-N16R8)│  │  ESP-ARM            │
            │                      │  │                     │
            │  4x BLD120A wheels   │  │  BTS7960 clamp      │
            │  4x AS5600 encoders  │  │  limit switches     │
            │  BNO085 IMU          │  │  current sense      │
            │  4x VL53L0X ToF      │  │                     │
            │  wheel current sense │  │                     │
            │  E-stop input        │  │                     │
            │  ── watchdog ──      │  │  ── watchdog ──     │
            └──────────────────────┘  └─────────────────────┘
                        │                       │
                        └────── E-STOP LOOP ────┘
                          (hardware, cuts motor
                           power without asking
                           the Pi or the ESPs)
```

## The load-bearing rule

> **The Pi is allowed to be late. It is not allowed to be trusted.**

A Raspberry Pi 5 runs Linux. Linux stalls — page faults, USB bus resets, a kernel task holding a
lock, the SD card blocking on a write, your own Python hitting a GC pause. None of these are faults
you can design out; they are properties of the platform. So every safety property must hold *while
the Pi is silent*.

Three mechanisms, all of which live below the Pi:

1. **Hardware E-stop loop.** The E-stop cuts motor power through a contactor, in copper. Both ESPs
   *read* the loop state so they can report it, but neither ESP and certainly not the Pi is in the
   path that de-energises the motors. GPIO47 on ESP-BASE stays as a sense input only.

2. **Command watchdog on each ESP.** If an ESP has not received a well-formed command frame in
   **100 ms** (5 missed ticks at 50 Hz), it brakes, drops `EN`, and latches into a `LINK_LOST` state
   that requires an explicit `RESUME` to leave. This is the single most important piece of firmware
   in the new architecture — write it first, test it by unplugging the USB cable.

3. **Fresh-authorisation clamping.** `SafetyMonitor`'s "confirmed" gate now has to cross a serial
   link to reach the arm. A boolean in a buffered message is not a safety gate — a delayed or
   replayed frame could authorise a clamp on a robot that has since moved. ESP-ARM must therefore
   require a `CLAMP` command carrying a **monotonic sequence number and a Pi timestamp**, and must
   refuse to act if the timestamp is older than 100 ms or the sequence number is not greater than
   the last one accepted. Stale authorisation is *not* authorisation.

Note what this preserves: your README's safety principle — *the clamp must not engage unless the
alignment logic confirms the platform is centred* — is unchanged. The alignment logic simply moved
to the Pi, and the gate got a freshness requirement to survive the move.

## What moves, what stays, what dies

| Component | Today | After |
|---|---|---|
| `CornerEdgeDetector` | ESP | **Pi** — port the C++ or reimplement in Python; keep the unit tests |
| `DockingStateMachine` | ESP | **Pi** |
| `MecanumDrive` | ESP | **Pi** (Pi sends 4 wheel setpoints, not body-frame velocity) |
| `DeadReckonOdometry` | ESP | **retired** — replaced by encoder + IMU fusion on the Pi |
| `SafetyMonitor` | ESP | **both** — authoritative copy on the Pi, reflex copy on each ESP |
| `Bld120aMotor` | ESP | **ESP-BASE**, unchanged |
| `Vl53l0xMux` | ESP | **ESP-BASE**, unchanged |
| `Bts7960Clamp` | ESP | **ESP-ARM** |
| `GpioLimitSwitches` | ESP | **ESP-ARM** |
| E-stop | ESP GPIO | **hardware loop**, sensed by both ESPs |

`lib/ports` and `lib/fakes` survive intact — the ports are still the right seams, they just have a
serial link running through some of them now. Your host unit tests in `test/` keep passing on
whatever logic stays in C++.

### Do not delete `DeadReckonOdometry` yet

Keep it as the fallback estimator. When an encoder magnet drifts out of its band or the BNO085 stops
reporting, degrading to "integrate the command" is much better than having no pose at all. Make it
the `IOdometry` implementation your fusion falls back to, and log loudly when it does.

## I²C layout on ESP-BASE — the nice result

Address survey:

| Device | Address | Count |
|---|---|---|
| VL53L0X ToF | `0x29` | 4 (identical) |
| AS5600 encoder | `0x36` | 4 (identical) |
| BNO085 IMU | `0x4A` | 1 |
| TCA9548A mux | `0x70` | 1 |

Four identical ToF and four identical encoders sounds like it needs eight mux channels. It does not.
**`0x29` and `0x36` do not collide**, so each mux channel can carry one ToF *and* one encoder — and
since each corner physically has both a wheel and a ToF sensor, the channel map falls out of the
chassis geometry:

```
TCA9548A channel 0  ->  FL:  VL53L0X (0x29) + AS5600 (0x36)
                 1  ->  FR:  VL53L0X (0x29) + AS5600 (0x36)
                 2  ->  RL:  VL53L0X (0x29) + AS5600 (0x36)
                 3  ->  RR:  VL53L0X (0x29) + AS5600 (0x36)
              4..7  ->  spare
```

This keeps `cfg::kMuxChannels = {0,1,2,3}` exactly as it is, and means one 4-wire bundle per corner
instead of two.

### Put the BNO085 on UART-RVC, not I²C

The BNO085 is notorious over I²C: it clock-stretches, and the ESP32 I²C peripheral handles long
clock stretching badly — you get intermittent timeouts that look like random sensor dropouts and
cost days to diagnose. **Use UART-RVC mode instead.** Tie `PS0`/`PS1` for RVC, and the part streams
19-byte frames at a fixed 100 Hz containing yaw/pitch/roll and linear acceleration, one direction
only, no handshaking. It costs one GPIO on ESP-BASE and removes an entire category of bug.

The trade-off: RVC gives you Euler angles, not the full quaternion or calibration status. For a
planar indoor robot where you mostly want **yaw rate and heading**, that is all you need. If you
later want the full rotation vector, move to SPI rather than back to I²C.

### Encoder rate budget

Per encoder read through the mux: 1-byte channel select + 2-byte angle register read ≈ 100 µs at
400 kHz. Four encoders ≈ 400 µs per tick. At 100 Hz that is 4 % bus utilisation — comfortable, even
sharing the bus with ToF ranging.

**Watch for:** AS5600 reads raw angle 0–4095 that wraps. Unwrapping must happen on ESP-BASE at the
sample rate, not on the Pi from downsampled data — if a wheel turns more than half a shaft
revolution between samples the Pi cannot recover the direction. ESP-BASE sends **accumulated
counts**, never raw angle. And remember the lesson already recorded in `platformio.ini`: the AS5600
sits upstream of the 15:1 gearbox, so its counts are shaft counts, 15× wheel counts.

## Pin budget on ESP-BASE

Moving the clamp to ESP-ARM frees seven pins: `11, 12, 13` (BTS7960), `1, 2` (clamp current ADC),
`14, 21` (limit switches).

New requirements: BNO085 UART RX (1 pin), wheel current sense (1 ADC pin). Both fit in the freed
set — put the current sense on GPIO1 or GPIO2, which are ADC1 and already routed for analogue.

Net result: ESP-BASE has *more* free GPIO after this change than before. The existing reserved
ranges are unaffected — GPIO26–37 stay off-limits for the octal PSRAM, GPIO19/20 stay on native
USB-CDC, which is now the Pi link.

## Serial protocol

One protocol, both ESPs, newline-delimited JSON. JSON because you already depend on ArduinoJson,
it's debuggable by eye with a terminal, and the schema maps 1:1 onto ROS 2 messages when you get
there. At 50 Hz with ~200-byte frames you need ~10 kB/s per link; **use 921600 baud**, not 115200 —
at 115200 a 200-byte frame takes 17 ms, which eats most of a 20 ms control period.

### Pi → ESP-BASE, every tick

```json
{"seq":1042,"t":1754300000123,"cmd":"DRIVE","w":[0.31,0.28,0.31,0.28],"en":true,"brk":false}
```

Wheel setpoints, not body velocity — mecanum mixing happens on the Pi so wheel-level compensation
(slip, per-wheel calibration from your `MotorCalAnalysis` work) applies where the estimator lives.

### ESP-BASE → Pi, every tick

```json
{"seq":1042,"t":88123,"enc":[120345,119880,120401,119795],"imu":{"yaw":0.031,"pitch":0.001,"roll":-0.002,"gz":0.004},
 "tof":[{"mm":142,"ok":true},{"mm":139,"ok":true},{"mm":0,"ok":false},{"mm":401,"ok":true}],
 "amps":1.82,"alarm":false,"estop":false,"link":"OK"}
```

Echoing `seq` back is what lets the Pi measure true round-trip latency and detect a stalled link
from the data itself.

### Pi → ESP-ARM

```json
{"seq":1042,"t":1754300000123,"cmd":"CLAMP","auth":{"confirmed":true,"pose_age_ms":18}}
```

`ABORT` and `UNCLAMP` need no authorisation and must always be honoured. `CLAMP` is rejected unless
`confirmed` is true, `t` is within 100 ms of ESP-ARM's estimate of now, and `seq` exceeds the last
accepted.

### ESP-ARM → Pi

```json
{"seq":1042,"t":45102,"state":"CLOSING","limit_open":false,"limit_closed":false,"amps":2.4,"estop":false,"link":"OK"}
```

### Why this is "ROS-ready"

These four message shapes are deliberately isomorphic to `sensor_msgs/JointState`,
`sensor_msgs/Imu`, `sensor_msgs/Range` and a small custom clamp message. Migrating to ROS 2 later
means writing a bridge node that reads these same frames and republishes them — **zero firmware
change**. That is the whole reason to define the protocol carefully now rather than letting it grow
organically.

## Clock discipline

Three clocks that do not agree: Pi wall time (ms since epoch), and each ESP's `millis()` since its
own boot. The `t` field means different things in each direction, deliberately — Pi-origin frames
carry Pi time, ESP-origin frames carry ESP uptime.

The Pi maintains an offset estimate per ESP from the `seq` echo round-trip, and timestamps all
logged sensor data in Pi time. Do not attempt NTP-style sync across USB serial; a simple offset with
a minimum-round-trip filter is accurate to a millisecond or two, which is well inside a 20 ms tick.

## Migration order

Each step leaves you with a robot that works. Do not skip ahead.

1. **Watchdog first.** Add the 100 ms link watchdog and `LINK_LOST` latch to the *existing* firmware,
   with the Pi doing nothing but sending heartbeats. Prove it by yanking the cable while the wheels
   turn. Nothing else happens until this is trustworthy.
2. **Telemetry up.** ESP-BASE streams the sensor frame; the Pi only logs it. Verify encoder counts
   track hand-turned wheels and IMU yaw tracks the chassis. No control authority yet.
3. **Estimator offline.** Build the encoder+IMU fusion on the Pi against *recorded* logs from step 2.
   This is where replay earns its keep — tune without a robot.
4. **Control down.** Pi takes over wheel setpoints. Docking logic still disabled; teleop only. Your
   existing `teleop.py` is the natural starting point.
5. **Sequencer across.** Port `DockingStateMachine` to the Pi, driven by the Pi's estimator.
6. **Arm last.** Split ESP-ARM out and implement fresh-authorisation clamping. Last because it is
   the only step where a bug can crush something.

## Open questions

- **Wheel current sense** — one aggregate channel or four? Aggregate is cheaper and enough for a
  stall trip; per-wheel is what you need to detect a single seized caster, which is exactly the
  failure mode `docs/concepts/2026-07-14-caster-realignment-current-wiggle.md` is about.
- **Does ESP-ARM need its own E-stop sense**, or is the hardware loop plus the Pi's `ABORT` enough?
  Leaning yes — it is one GPIO and the arm is the dangerous end.
- **Pi 5 boot medium** — NVMe via the PCIe HAT is dramatically more reliable than SD under the write
  load of continuous logging. SD cards on a robot that gets power-cycled abruptly do corrupt.
