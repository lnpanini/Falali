# Session snapshot — 2026-08-04 to 2026-08-06

What changed, what was measured, and what was got wrong along the way. Written so
a future session can pick up without re-deriving anything.

**Headline:** the drivetrain works. Four BLD-120A motors run from an ESP32-S3 over
hand-built transistor adapters, driven by an Xbox controller over BLE, with
direction calibration done and encoders + ToF reading through an I²C mux.

---

## 1. Architecture change

The project moved from "the ESP32 is the entire control stack" to a **Raspberry
Pi 5 brain driving two ESP32s over USB serial**, prompted by adding 4 encoders and
an IMU — state estimation wants a filesystem, log replay and a language you can
iterate in.

Spec: `docs/superpowers/specs/2026-08-04-rpi5-main-controller-design.md`
Runbook: `docs/rpi5-setup.md`

The load-bearing rule, and the reason `LinkWatchdog` exists:

> **The Pi is allowed to be late. It is not allowed to be trusted.**

Linux stalls — page faults, USB resets, GC pauses. The failure that matters is not
"the Pi crashed" but "the Pi went quiet while the last command was DRIVE FORWARD",
because motor drivers hold their last value indefinitely. So the ESP requires
continuous proof someone is in charge, and brakes when it stops arriving.

---

## 2. What was measured (and what it replaced)

| Fact | Method | Replaced |
|---|---|---|
| **SV is a high-impedance voltage input** | diode test read open both ways | "pot-wiper drags the source down" |
| **The 2.59 V ceiling was the ESP32's DAC**, not the driver | provenance check — 2026-07-15, WROOM-32D, `dacWrite` | a planned MCP4728/op-amp/level-shifter purchase |
| **Break-away ≈ 5% duty** | encoder sweep, no load | an estimated 24%, which implied a gearing redesign |
| **EN/BRK/F-R are optocouplers idling ~5 V**, pull-up ≈10 kΩ, ≈440 µA | powered measurement + manual | assumption of 2.2 kΩ |
| **Driver takes PWM natively** | manual, Speed Command mode C | DAC / op-amp / RC filter |
| **PWM range is 1–3 kHz** (SYS manual) vs 1–10 kHz (other manual) | two manuals disagree | 20 kHz, then 5 kHz — both out of spec |
| **P-sv is the overload limit, not a speed trim** | manual | bench notes calling it a speed trim |
| **There is no ALM terminal** | manual + terminal count | a `motor_alarm` input that always read "no fault" |
| **Direction: FL/RL inverted, FR/RR not** | `m` identification routine | uncalibrated |

### The one that mattered most

`docs/BLD-120-English-version.pdf` and the SYS manual disagree, and the SYS one
has the graph that killed a redesign: **duty/speed is linear from 4% duty = 4%
speed — no dead zone.** The measured 5% break-away confirms it. Minimum speed is
therefore ~5% of top speed, comfortably *below* the 300 mm/s target rather than
above it.

---

## 3. Corrections made during the session

Recorded because each was believed and acted on before being disproved.

1. **`pins.h` did not match the fabricated PCB.** It held pre-PCB design intent —
   SV on the ToF XSHUT pins, I²C on two BRK lines. Rebuilt from the KiCad netlist.
2. **GPIO35/36/37 were NOT in use.** Claimed from a rendered schematic; the
   netlist shows them unconnected. No bodge wires needed.
3. **ACS758 is the right current sensor, not oversized.** The "25× better
   resolution" claim for an ACS712 was wrong — Hall noise scales with sensitivity,
   so the real gap is 1.5×. And a 5 A part clips during the 8 A stall and 30 A
   inrush, which are the events worth measuring.
4. **A pulldown cannot replace the transistor.** Pin-safe needs ≤19.4 kΩ,
   opto-off needs ≥32 kΩ. No overlap.
5. **10 kΩ was the wrong SV series resistor**; 1.5 kΩ, because it divides against
   the driver's unmeasured input impedance.

### The bug that cost an evening

The first transistor adapter was built with **C and E swapped** and the **100 kΩ
on the collector instead of the base**. Signature: base stuck at 1.678 V instead
of clamping at ~0.7 V; collector barely moving between states (1.069 → 0.945 V).
The driver showed a **green LED throughout** — no fault, because from its side it
had simply been told to stop. Full write-up in `docs/hardware-architecture.md` §4.

---

## 4. What was built

### Firmware
- `lib/domain/LinkWatchdog.{h,cpp}` + 17 host tests — dead-man's handle on the Pi
  link, with a latch so a recovered link does not silently resume a stale plan.
- `src/bench_s3_motor.cpp` — 4-motor serial bench rig: per-wheel control, motor
  identification (`m`), direction calibration (`v`/`p`), settle-aware speed sweep
  (`k`), I²C scan (`i`), and the Pi's `W` setpoint protocol with a 300 ms watchdog.
- `src/bench_bld_drive.h` — shared BLD-120A drivetrain (pin map, transistor
  inversion, PWM, direction calibration).
- `src/bench_i2cmux.h`, `src/bench_tof4.h` — PCA9548A mux and 4× VL53L0X behind it.
- `bench_ble/main/sketch_bld.cpp` — Xbox gamepad → BLD-120A, with pairing control,
  the same calibration commands, and ToF readout.

### Pi side
- `pi/bridge.py`, `pi/link.py` — async serial bridge with link-age tracking.
- `pi/mecanum.py` — port of `MecanumDrive.cpp` with sign-convention assertions.
- `pi/gamepad.py`, `pi/drive_gamepad.py` — evdev Xbox reader → mixing → setpoints.
- `pi/fake_esp.py` — simulated ESP for testing the bridge with no hardware.
- `tools/find_pi.sh`, `tools/pi_bootstrap.sh` — locate and provision the Pi.

---

## 5. Sign conventions — the trap in this codebase

Two mixers with **opposite** conventions, both correct:

```
bench_mix.h        fl = vx + vy + w        vy+ = strafe RIGHT, w+ = CW
MecanumDrive.cpp   fl = vx - vy - omega    vy+ = strafe LEFT,  w+ = CCW
```

They cancel: the signs differ *because* the conventions differ. `pi/mecanum.py`
follows `MecanumDrive.cpp`; `bench_bld_drive.h` follows `bench_mix.h`.
**Do not "fix" one without the other.**

---

## 6. Open items

- [ ] **Top speed** — the last number needed. `max_lin_mm_s = 300` in `config.h`
      is wrong; actual is several times higher, so `DeadReckonOdometry`
      under-estimates every distance.
- [ ] **Which fix made the motor run** — brake latched, P-sv maxed, or 5→2 kHz?
      Bisect before building three more adapters.
- [ ] **ACC/DEC pot direction** — 0.3 s or 15 s at fully-CCW is still unknown, and
      a slow ramp corrupts sweeps (the sweep now detects this and marks rows
      `NOT-SETTLED`).
- [ ] **`main.cpp` still passes one shared EN/BRK pair** to all four motors; the
      board has eight separate lines.
- [ ] **Verify FR and RL SV/BRK wiring** — two rows of the breadboard pin table
      were transcribed in a different order from the other two.
- [ ] **Mecanum odometry calibration** — rim speed is not ground speed. Needs
      empirical scale factors, separately for forward, strafe and rotation.
- [ ] `bench_s3_motor.cpp` still has its own mux code; migrate to
      `bench_i2cmux.h` so there is one `g_mux_cur`.

---

## 7. Bench state at the end of the session

| | |
|---|---|
| Drivetrain | 4× BLD-120A, transistor adapters, all directions calibrated |
| Control | Xbox controller over BLE to the ESP32-S3 (Bluepad32) |
| Encoder | 1× AS5600 on mux ch0, reading RPM |
| ToF | 4× VL53L0X wired to mux ch4–7 |
| Pi | provisioned but flaky on a phone hotspot; not in the loop for the bench |
| Tests | 8 host suites, 68 tests, green under `-Wall -Wextra -Werror` |
