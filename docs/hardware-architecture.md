# Falali — hardware architecture and parts list

**Last updated:** 2026-08-05. **Partially superseded 2026-08-16** — see below.
**Authoritative sources:** `Wheel Drive PCB.net` (Eeschema 9.0.6, 2026-08-05) for every
GPIO assignment; `docs/BLD-120-English-version.pdf` for driver behaviour; bench
measurements dated inline.

This document exists because the same facts kept getting re-derived from rendered
schematics and stale notes, and twice got derived wrong.

> ## ⚠️ Read `docs/hardware/` first
>
> As of **2026-08-16** the handover documentation lives in
> [`docs/hardware/`](hardware/), and it was built by parsing the KiCad netlists
> of **all three** boards directly:
>
> | | |
> |---|---|
> | [`hardware/startup.md`](hardware/startup.md) | power tree and power-up runbook — **none of this is in any KiCad file** |
> | [`hardware/components.md`](hardware/components.md) | parts, specs, and what is worth salvaging |
> | [`hardware/boards.md`](hardware/boards.md) | every connector pinout on all three boards, netlist-verified |
> | [`hardware/fabrication.md`](hardware/fabrication.md) | re-ordering, modifying and repairing the boards |
>
> **Where this file and `docs/hardware/` disagree, `docs/hardware/` is newer**
> and says which netlist line it came from. Sections below that have been
> superseded are marked inline. The GPIO tables in §2 were re-checked against the
> netlist on 2026-08-16 and are **correct** — it is the surrounding commentary
> that had drifted.

---

## 1. System topology

> **⚠️ This diagram is stale in two places.** Corrected version below it.

```
Raspberry Pi 5  ──USB──  ESP-BASE (Wheel Drive PCB)  ──  4× BLD-120A  ──  4× BLDC motor
   (brain)                 ESP32-S3-DevKitC-1 N16R8        (24 V)          (15:1 gearbox)
                                  │
                                  ├── 4× VL53L0X ToF         (XSHUT re-addressing)
                                  ├── 1× BNO085 IMU
                                  ├── 1× ADS1115 ← 4× ACS758LCB current sensors
                                  │
                                  └──UART1 (GPIO1/2)── ESP-ARM ── clamp + flippers

                                       4× AS5600 encoder  [REMOVED 2026-08-13]
```

**As actually built, 2026-08-16:**

```
  Xbox BLE gamepad
        │ (Bluepad32, BLE)
        ▼
  ESP-BASE (Wheel Drive PCB) ── J4–J7 ── 4× Driver PCB ── 4× BLD-120A ── 4× BLDC
  ESP32-S3-DevKitC-1 N16R8              (3V3→open-collector)   (24 V)   (15:1, 150 mm)
        │
        ├── 4× VL53L0X ToF     (XSHUT re-addressing, GPIO4-7)
        ├── 1× BNO085 IMU      (0x4A)
        ├── 1× ADS1115 (0x48)  ← 4× ACS758LCB current sensors
        │
        └── ESP-NOW, WiFi ch 1 ──► ESP-ARM (Arm Subsystem PCB)
                                     ├── 2× BTS7960 ── wormgear extension (12 V)
                                     ├── PCA9685 (0x40) ── 4× flipper servo (5 V)
                                     ├── 4× VL53L0X ToF  (0x30–0x33)
                                     └── 8× limit switch

  4× AS5600 encoder  [REMOVED 2026-08-13 — no wheel-speed feedback of any kind]
```

**Two corrections to the original diagram:**

1. **The base↔arm link is ESP-NOW, not UART on GPIO1/2.** The UART never passed
   a byte and was retired 2026-08-14 — §5 of this document already records
   this, the diagram was simply never updated.
2. **The Raspberry Pi 5 is not in the control loop.** The gamepad connects
   directly to the base ESP over BLE and the docking state machine runs on the
   base. The Pi-5-as-brain design (decision log, 2026-08-04) was superseded by
   the ESP32-S3-only prototype. `pi/` remains in the repo but is not part of the
   running system. **[HYPOTHESIS — inferred from the firmware's structure and
   the ESP-NOW move; nobody wrote down the moment the Pi was dropped.]**

The encoders never worked on the fabricated board and have been abandoned. There
is now **no wheel-speed feedback of any kind** — the only drivetrain fault signal
is per-wheel current (`cfg::kWheelStallAmps`).

See `docs/superpowers/specs/2026-08-04-rpi5-main-controller-design.md` for the
control split and the link watchdog.

---

## 2. GPIO map — ESP32-S3-DevKitC-1 **N16R8**

Extracted from the netlist. Mirrored in `include/pins.h`; that file is the code's
copy of this table.

### Wheel motors (index order FL, FR, RL, RR — matches `fal::Corner`)

| Signal | FL | FR | RL | RR |
|---|---|---|---|---|
| **SV** (speed, PWM) | 42 | 21 | 18 | 10 |
| **F/R** (direction) | 41 | 47 | 17 | 11 |
| **EN** (enable) | 40 | 48 | 16 | 12 |
| **BRK** (brake) | 39 | 38 | 15 | 13 |

**EN and BRK are per-wheel, not ganged.** The pre-PCB design assumed one shared
pair; the fabricated board gives each wheel its own. Independent shutdown is
strictly better, but `main.cpp` must pass each motor its own pins.

**There is no ALARM net.** The BLD-120A exposes no ALM terminal — "RUN/ALM" is an
LED. `pins::kWheelALARM == kNoPin`.

### Sensors

| Signal | GPIO | Note |
|---|---|---|
| ToF XSHUT FL/FR/RL/RR | 4, 5, 6, 7 | per-sensor reset for address assignment |
| I²C SDA / SCL | 8 / 9 | one bus for everything |
| *(none)* | 1, 2, 3, 14 | freed by the encoder removal; the arm link went to ESP-NOW 2026-08-14 |

### Reserved and free

| Pins | Status |
|---|---|
| 26–37 | flash + octal PSRAM — **board correctly leaves 35/36/37 unconnected** |
| 19, 20 | native USB — unconnected |
| 0, 45, 46 | strapping — unconnected |
| 1, 2, 3, 14 | free — 3 is strapping (JTAG select), 14 is ADC2 and **`bench_ble` now enables WiFi**, so treat 14 as digital-only |
| **43, 44** | UART0 — free **only** in a native-USB-CDC build. `bench_ble` is not one (`CONFIG_ESP_CONSOLE_UART_NUM=0`) |

26 GPIO used, no duplicates, no reserved-pin conflicts. Verified programmatically.

---

## 3. BLD-120A driver interface

### Speed (SV)

The driver has a **native PWM speed input**: manual, Speed Command mode C —
*"speed can be adjusted by PWM control between 1KHz~10KHz, motor speed is
influenced by duty."* No DAC, op-amp or RC filter needed.

- ~~`kSvPwmFreqHz = 5000` — **1–10 kHz is a hard spec range**, not a preference~~
  **[SUPERSEDED — resolved 2026-08-16]** The actual value is
  **`kSvPwmFreqHz = 2000`**, and the correct range is **1–3 kHz**. The
  BLD-120A-specific spec sheet (`docs/BLDC-BLD120A-bldc-motor-controller-specs.pdf`)
  states *"The pulse frequency range: 1-3KHz"*, agreeing with
  `SYS-BLD-120A-manual.pdf` against `BLD-120-English-version.pdf`'s 1–10 kHz.
  **The 5 kHz this document used to specify was out of spec.** `pins.h` is right.
  The datasheet also advises a **2 %–90 % duty range**, with 90 % duty giving
  maximum speed.
- 1–10 kΩ series resistor between GPIO and SV (manual FAQ A). 1.5 kΩ in use,
  fitted as R7 on the Driver PCB.
- ~~**SV is a high-impedance voltage input.**~~ **[SUPERSEDED — treat as
  UNVERIFIED]** The diode test of 2026-08-05 read open in both directions, but a
  later powered test on the same terminals gave conducting readings, and the two
  cannot both be right. `pins.h` records this contradiction; this section
  originally reported only the first result. It does not matter in practice —
  3.3 V PWM at 2 kHz drives all four motors through the adapters, verified by
  behaviour. Re-measure only if you need the driver's true input model for a
  redesign.

  What *is* still established: the old "pot-wiper drags it down" claim and the
  2.59 V figure behind it were the ESP32's weak DAC on the WROOM-32D bench rig,
  not the driver.

### Control lines (EN / BRK / F-R) — opto-isolated, idle near 5 V

Each is an optocoupler LED fed from the driver's internal rail. Measured
2026-08-05: pull-up ≈10 kΩ, ≈440 µA when pulled to COM. **The ESP32-S3 is not
5 V tolerant** — these must never connect straight to a GPIO.

### Polarity (bench-confirmed 2026-07-15, datasheet-confirmed 2026-08-05)

| Line | Asserted state at the driver pin | Meaning |
|---|---|---|
| EN | LOW (connected to COM) | motor enabled |
| BRK | LOW (connected to COM) | brake applied |
| F/R | HIGH (released) | forward |

### Onboard trims — both matter

| Trim | Function | Required setting |
|---|---|---|
| **RV** | speed pot, sums with external input | **fully anticlockwise (left)** — external speed control *fails* otherwise (manual, stated 3×) |
| **P-sv** | overload trip point. Marked in **watts (30–120 W)** on one revision, **amps (1.6–8 A)** on another — same setting | **5 A ≡ 120 W at 24 V.** Too high *or* too low trips the red LED and the motor won't run |
| **ACC/DEC** | acceleration & deceleration ramp, **0.3–15 s** | **minimum (0.3 s)** |

P-sv is the *only* hardware overload protection on this drivetrain. It is not a
speed trim, despite what earlier bench notes claimed.

> **⚠️ [ADDED 2026-08-16] This table used to list two trims. There are three.**
> The **ACC/DEC** potentiometer ramps every speed change over anywhere from 0.3 to
> 15 seconds, and **the positions of all four drivers are unrecorded.** A long
> ramp would invalidate `DeadReckonOdometry`'s assumption that the wheel follows
> the command immediately, and would be indistinguishable from stiction. See
> [`hardware/startup.md`](hardware/startup.md) §6. **[VERIFIED — the trim exists
> and its range; HYPOTHESIS — that it explains any observed behaviour.]**

---

## 4. Transistor adapter (×4) — required between PCB and driver

Converts 3.3 V push-pull GPIO into an open-collector pull to COM, and keeps the
driver's ~5 V rail off the ESP.

```
GPIO ──/\/\/\── 10 kΩ ──┬── Base
                        │
                     100 kΩ          ← gate/base pulldown: CRITICAL.
                        │              Holds the transistor OFF while the GPIO is
                       COM             Hi-Z at boot, reset, or unplugged.
Collector → driver input
Emitter   → COM
```

Pass-through, same pin order both sides (`BRK · EN · F/R · COM · SV`):

| Pin | Treatment |
|---|---|
| 1 BRK, 2 EN, 3 F/R | transistor as above |
| 4 COM | direct wire |
| 5 SV | 1.5 kΩ series resistor |

**The transistor inverts every control line.** Handled in firmware by the single
switch `fal::kControlViaMosfet` in `Bld120aMotor.h`, which derives all four
polarity constants and forces push-pull gate drive. Verified by `static_assert`.

**Device:** **2N3904** (NPN). Base current (3.3 − 0.7)/10 k = 260 µA against a
440 µA load — saturates hard.

> ⚠️ **BC547 is C-B-E; 2N3904 is E-B-C** (flat face toward you, legs down).
> Identical packages, mirrored pinouts.
>
> **[UPDATED 2026-08-16]** This is no longer a free choice. The adapter is now a
> fabricated board — `KiCad/Driver PCB/` — whose footprint is
> `TO-92_Inline_Wide_**EBC**` with a 2N3904 symbol. **A BC547 dropped into those
> holes unrotated swaps collector and emitter** and reproduces exactly the fault
> described below. Use 2N3904.

> **[SUPERSEDED 2026-08-16]** The hand-wired adapter described in this section
> was fabricated as the **Driver PCB** (42.8 × 33.8 mm, ×4). Component values are
> unchanged — 10 kΩ base, 100 kΩ pulldown, 1.5 kΩ on SV — and the pin order still
> matches Wheel Drive J4–J7. See [`hardware/boards.md`](hardware/boards.md) §2
> for the netlist-derived schematic and the per-channel diagnostic procedure.

### The wiring error that cost an evening (2026-08-05)

The first adapter was built like this, and the motor would not run at all:

```
WRONG                                RIGHT
ESP ──10k── B                        ESP ──10k──┬── B
            C ──100k── COM                      │
            E ───────── driver                100k
                                                │
                                               COM
                                                C ───────── driver
                                                E ───────── COM
```

Two faults at once: **C and E swapped**, and the **100 kΩ on the collector instead
of the base**. The transistor did nothing, EN never asserted, and the driver sat
happily idle showing a green LED — no alarm, because from its point of view it had
simply been told to stop.

**Diagnostic signature, for next time:**

| Measurement | Healthy | What we saw |
|---|---|---|
| base → COM, GPIO high | **~0.7 V** (junction clamps) | **1.678 V** — junction not conducting |
| collector → COM, enabled vs disabled | huge change | **1.069 → 0.945 V** — barely moves |

A base that will not clamp at ~0.7 V means the B-E path to COM is broken. A
collector that does not change between states means the transistor is not
switching. Nothing was damaged — currents were microamps throughout.

---

## 5. I²C bus — one bus, shared by everything

| Device | Address | Collision handling |
|---|---|---|
| 4× VL53L0X ToF | `0x29` ×4 | **XSHUT re-addressing** — held in reset, brought up one at a time (`Vl53l0xArray`) |
| TCA9548A | `0x70` | fitted for the (now removed) encoders; still reported by `src/pcb_identify.cpp` |
| BNO085 IMU | `0x4A` | — |
| ADS1115 | **`0x48`** | ADDR → GND. **Do not leave floating or tie to SDA** — that gives `0x4A` and collides with the IMU |

### Base ↔ ESP-ARM link: ESP-NOW

The three-wire UART on GPIO1/2 never passed a byte and was retired **2026-08-14**.
Commands now go by radio.

| | |
|---|---|
| Transport | ESP-NOW, WiFi **channel 1**, STA mode, no AP |
| Payload | plain text, **NUL-terminated** (`strlen(cmd) + 1`) |
| Base MAC | `14:C1:9F:3B:7B:E4` — checked at boot against `SELF_ESP_MAC` |
| Arm MAC | `3C:DC:75:5C:8B:08` |

**Four commands, and no more.** The arm is already flashed to parse exactly
these, so the strings are an interface, not an implementation detail:

```
grab        release        home_setup        estop
```

Replies arrive as text on a callback: `done_grab`, `done_release`, `done_home`,
`busy`, `stopped`, `failed:<reason>`, `rejected:<reason>`.

**What this cost.** Bluepad32/BTstack owns the 2.4 GHz radio for the gamepad, and
WiFi STA now shares it through the coexistence scheduler. Expect the pad to be
marginally less responsive and to drop slightly more often than on the UART
build. That is a real price paid to avoid three wires — if it becomes
intolerable, the answer is to fix the wiring, not to tune the radio.

**Busy interlock.** `busy` from the arm suppresses the *next* ordinary command
and then clears, so one press gets one refusal rather than a queue that fires
against a state that has since changed. E-stop is exempt and always sends: an
E-stop a status message can suppress is not an E-stop.

**Gone with the UART.** The arm's per-axis jog (`e`/`r`/`E`/`R`) and flip
(`f`/`F`) characters are not part of the four-command protocol, so D-pad arm jog
and the LT/RT flips no longer exist. The D-pad is drive-only.

### Encoders — removed 2026-08-13

Four AS5600 on a TCA9548A mux, never made to work: all four share address `0x36`
with no address pin, the board commons SDA/SCL, and isolating them needed eight
trace cuts at the encoder connectors. Resistance checks found two modules at
60 kΩ and 165 kΩ against 280 kΩ for the healthy pair, and the parasitic-power
signature pointed at a missing VCC connection.

> **[CORRECTED 2026-08-16] The missing VCC is on the PCB, not in the loom.**
> `pin 1` of every 1×04 encoder connector — J1, J11, J15, J18 — is unconnected in
> the netlist. That header carries SDA, SCL and GND only. The AS5600's sole power
> path was the separate 1×03 analog connector, which is exactly the parasitic-power
> signature that was observed. No amount of re-looming would have fixed it.
> **[VERIFIED — netlist]**
>
> A later attempt used an external PCA9548-type mux; it overheated and failed with
> more than two AS5600 attached. **[OBSERVED — Bryan]**

The pin tables are deleted from `pins.h` rather than commented out. GPIO3 and
GPIO14 are now free; GPIO1 and GPIO2 belong to the arm link.



---

## 6. Current sensing

**4× ACS758LCB → ADS1115 (4-channel, 16-bit) → I²C**

This feeds `StallDetector`, which — with no ALM terminal and P-sv as the only
hardware trip — is the main software protection for the drivetrain.

### Why the ACS758 is the right choice — headroom beats sensitivity

The driver is rated **8 A continuous, 30 A instantaneous (<3 s)** (manual p.2).
Sizing the sensor to the *running* current (~0.2–1 A) leaves it blind during
exactly the events worth measuring.

Both candidates, ratiometrically scaled to the board's 3.3 V rail:

| Part | Sensitivity | Noise (current-referred) | Output clips at |
|---|---|---|---|
| **ACS758LCB-050B** | 26.4 mV/A | 0.175 A rms | **±62 A** |
| ACS712-05B | 122 mV/A | 0.114 A rms | **5 A** |

**The resolution gap is 1.5×, not the ~25× a sensitivity comparison implies** —
Hall noise scales with sensitivity, so current-referred noise barely moves between
parts. An earlier revision of this document called the ACS758 "badly oversized".
That was wrong, and this section is the correction.

What each part actually sees:

| Condition | ACS758-050B | ACS712-05B |
|---|---|---|
| 0.5 A running | 1.66 V | 1.71 V |
| 1 A loaded | 1.68 V | 1.77 V |
| **8 A stalled** | 1.86 V | **CLIPPED** |
| **30 A inrush** | 2.44 V | **CLIPPED** |

A clipped sensor cannot tell 5 A from 30 A — the difference between "working hard"
and "something is badly wrong". For a signal feeding a fault trip, that matters far
more than resolving 100 mA at idle.

**Resolution is sensor-limited, not ADC-limited.** ADS1115 at PGA ±2.048 V gives
62.5 µV/LSB = **2.4 mA per count**, 74× finer than the sensor's own noise floor.
Averaging 8 samples brings that floor to ~62 mA — fine enough to spot a seized
caster from the current differential between wheels.

### Remaining choices

1. **Prefer `-050B` (bidirectional) over `-050U`.** DC-bus current is nominally
   one-directional, but applying BRK can push energy back toward the supply. `B`
   costs one bit of range and removes the question.
2. **Supply rail — CONFIRMED 3.3 V.** The board feeds J8/J9/J12/J14/J17 from
   **3.3 V** (`/3V Wheel`). ACS758 is ratiometric, so at 3.3 V a -050B gives
   26.4 mV/A with a 1.65 V zero point — read directly by a 3.3 V ADS1115, no
   divider, no level shifting. Powering it at 5 V instead puts up to 5 V into a
   3.3 V ADC input and needs a divider. ~~Confirm the rail before wiring.~~
   **[VERIFIED — netlist, 2026-08-16: pin 1 of all four connectors is on
   `/3V Wheel`.]**

   Likewise the **ADS1115 address is `0x48` fixed in copper** — J8 pin 5 lands on
   the module's ADDR pin and the board ties it to ground. There is no jumper to
   set and no way to get it wrong. **[VERIFIED — netlist]**
3. The sensor is *observation*, not protection. The real trip chain is the driver's
   P-sv overload trim, then the 10 A fuse, then software.

> ## ⚠️ [ADDED 2026-08-16] None of this is live — the sensors are not wired
>
> The four ACS758 are physically installed **inline with the negative side of the
> motor driver supply**, but their outputs are **not connected to
> J9/J12/J14/J17**. `StallDetector` is reading an ADS1115 that sees nothing.
>
> Everything in this section is a correct analysis of a signal path that does not
> currently exist. Combined with the absence of an ALM terminal and the retired
> encoders, **the base has no drivetrain fault detection at all** — the trip chain
> is P-sv and the fuse, full stop.
>
> Connecting them would be four wires. See `gotchas.md` Open Issue C and
> [`hardware/components.md`](hardware/components.md) §4. **[OBSERVED — Bryan]**

---

## 7. Bill of materials

### Board-mounted (not yet fitted)

| Qty | Part | Where |
|---|---|---|
| 4 | 1×03 male header 2.54 mm | J2, J10, J13, J16 |
| 8 | 1×04 male header | J1, J11, J15, J18, J9, J12, J14, J17 |
| 4 | 1×05 male header | J4–J7 |
| 4 | 1×06 male header | TOF5–8 |
| 2 | 1×10 male header | J8, J19 |
| 2 | 1×22 **female** header | U6 — **socket it, don't solder the DevKit down** |
| 1 | 2-pos 5.08 mm terminal block (Phoenix MKDS-1,5-2-5.08 or equiv.) | J3 |
| 4 | M3 screw + standoff | H1–H4 |

108 male positions ≈ three 40-pin breakaway strips; 44 female ≈ two strips.

### Modules

| Qty | Part | Stage |
|---|---|---|
| 1 | ESP32-S3-DevKitC-1 **N16R8** | move |
| 4 | BLD-120A driver + BLDC motor, 15:1, 150 mm wheel | move |
| 4 | transistor adapter (BC547/2N3904 + 10 k + 100 k + 1.5 k) | move |
| 2 | **TCA9548A** breakout | ~~sense~~ — encoders abandoned, mux no longer needed |
| 4 | AS5600 breakout | ~~sense~~ — **removed 2026-08-13**, see §5 |
| 4 | VL53L0X breakout — **must break out XSHUT** | dock |
| 1 | BNO085 breakout (10-pin) | dock |
| 1 | ADS1115 breakout | sense |
| 4 | ACS758LCB (see §6 open questions) | sense |

**To get wheels turning you need only the "move" rows** plus the J4–J7 headers,
the DevKitC socket, J3 and the standoffs.

---

## 8. Decision log

| Date | Decision |
|---|---|
| 2026-08-04 | Pi 5 becomes the brain; ESPs become I/O. Link watchdog + latch is mandatory. |
| 2026-08-05 | `pins.h` rebuilt from the netlist — the previous map was pre-PCB design intent and did not match the fabricated board (SV on the ToF XSHUT pins, I²C on two BRK lines). |
| 2026-08-05 | PWM resolution 8 → 12-bit; SV carrier 20 kHz → 5 kHz (20 kHz was outside the driver's 1–10 kHz spec). |
| 2026-08-05 | No DAC / op-amp / level shifter for SV — the driver takes PWM natively and SV is high-impedance. |
| 2026-08-05 | Encoders via TCA9548A rather than AS5600L, to keep the existing AS5600 stock. |
| 2026-08-13 | **Encoders abandoned.** Never worked on the fabricated board; no wheel-speed feedback, current sense is the only drivetrain fault signal. |
| 2026-08-13 | Base↔arm link on UART1 GPIO1/2 — *not* GPIO43/44, which `bench_ble`'s IDF console occupies. |
| 2026-08-14 | **Base↔arm moves to ESP-NOW** (channel 1, plain text). The UART never passed a byte. Accepts BT/WiFi coexistence on the gamepad radio as the price. |

## 9. Open items

- [ ] **Minimum speed.** Break-away ≈0.78 V → ~413 mm/s at the wheel, already
      above the 300 mm/s target, and one 20 ms tick moves 8.3 mm against an
      8 mm centring tolerance. Likely a gearing/wheel-diameter problem, not a
      firmware one. The driver's PID speed loop may rescue it — **settle with the
      REF+ → 1 kΩ → SV pot sweep.**
- [x] ~~`max_lin_mm_s = 300` is wrong; actual is ~1400 mm/s at full command.~~
      **Done.** `config.h:195` sets `c.max_lin_mm_s = 1231.0f`; the `300.0f` in
      `DeadReckonOdometry.h` is only the struct default and is overridden by
      `makeOdometryCal()`. Accuracy is still ~±6 % — `gotchas.md` Open Issue B —
      but docking was restructured so the park point no longer depends on it.
- [ ] `main.cpp` still passes one shared EN/BRK pair to all four motors. **The
      board gives each wheel its own** (§2), so this leaves independent per-wheel
      shutdown unused.
- [x] ~~ACS758LCB-**050B** confirmed as the right part (§6); still to confirm the
      supply rail is 3.3 V and set ADS1115 ADDR→GND for `0x48`.~~ **Both resolved
      from the netlist, 2026-08-16** — the rail is 3.3 V and `0x48` is fixed in
      copper. See §6.
- [ ] Confirm the motor's rated watts, to set P-sv. **Still open, and it blocks
      the start-up procedure** — see [`hardware/startup.md`](hardware/startup.md) §6.

**Added 2026-08-16, from the netlist cross-check:**

- [ ] The **red mushroom button is wired as an enable, not an E-stop.** One wire
      to fix. `gotchas.md` §18.
- [ ] The **Arm PCB's servo headers S1–S4 are dead in firmware** — it drives a
      PCA9685 that has no connector on the board. `gotchas.md` §17.
- [ ] **Two custom KiCad libraries are missing** (`My_30.007_Library`,
      `HengLi's Footprint Library`). Boards can be re-ordered as-is but not
      cleanly re-synced schematic→layout.
      [`hardware/fabrication.md`](hardware/fabrication.md) §3.
- [ ] The **BNO085 module pinout is not captured anywhere** — J19 exposes
      3V3·GND·SCL·SDA on a generic 10-pin header, and some common breakouts put a
      regulator *output* on pin 2. Verify before seating.
      [`hardware/boards.md`](hardware/boards.md) §1.

---

## Bring-up gotchas that cost real time (2026-08-06)

Two failures that both presented as something other than what they were. Both
are worth reading before debugging a silent or dead board.

### 1. Missing ESP-to-PSU ground presented as a GPIO0 strapping fault

**Symptom:** the ESP booted `rst:0x15,boot:0x3 (DOWNLOAD(USB/UART0))` on every
reset, so the application never ran. Uploads worked without touching BOOT, which
is itself the tell — a board that always boots to download never needs the button.

**Cause:** the ESP had no ground connection to the motor PSU. With no shared
reference, GPIO0 had nothing sane to sit at and read low at reset.

**Why it misleads:** everything points at GPIO0. The netlist leaves that pin
unconnected, so the obvious conclusion is a solder bridge, and you go looking for
copper faults. The isolating test is the fast one: lift the module off the PCB
and reset it standalone. `0x8` off the board and `0x3` on it localises the fault
to the board in one move.

**Boot modes worth memorising:**

| Code | Meaning |
|---|---|
| `boot:0x8 (SPI_FAST_FLASH_BOOT)` | normal — the app is running |
| `boot:0x3 (DOWNLOAD(USB/UART0))` | GPIO0 was low at reset; app never runs |

### 2. HWCDC blocks forever if the host stops reading

**Symptom:** the board goes permanently silent — no banner, no command response,
and esptool fails with `No serial data received`. Only a physical USB replug
clears it. Looks exactly like dead firmware or a dead board.

**Cause:** with `ARDUINO_USB_MODE=1`, `Serial` is HWCDC, which **blocks** when its
TX buffer fills and no host is draining it. Any sketch that prints unprompted
(a heartbeat, periodic telemetry) will fill that buffer the moment the terminal
closes, then hang inside `Serial.printf`.

**Fix, and it belongs in every S3 sketch that prints unprompted:**

```cpp
Serial.setTxTimeoutMs(0);   // never block; discard instead
```

Losing output to a terminal that is not attached is always the right trade
against wedging the controller. This is the same principle as `LinkWatchdog`,
one layer down: a peer that stops listening must never be able to stall us.

### 3. DTR is the IO0 strap on USB-Serial-JTAG

When scripting the serial port on an ESP32-S3, open it with **DTR and RTS LOW**.
Asserting DTR is esptool's "set IO0" step and forces the chip into download mode.
A probe that sets `dtr=True` will put the board into the bootloader and then
report it as silent — a self-inflicted fault that reads as a hardware one.
