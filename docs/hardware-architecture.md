# TrolleyBot — hardware architecture and parts list

**Last updated:** 2026-08-05
**Authoritative sources:** `Wheel Drive PCB.net` (Eeschema 9.0.6, 2026-08-05) for every
GPIO assignment; `docs/BLD-120-English-version.pdf` for driver behaviour; bench
measurements dated inline.

This document exists because the same facts kept getting re-derived from rendered
schematics and stale notes, and twice got derived wrong. **Anything here that
contradicts a comment elsewhere in the repo — trust this, or re-extract from the
netlist.**

---

## 1. System topology

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

The encoders never worked on the fabricated board and have been abandoned. There
is now **no wheel-speed feedback of any kind** — the only drivetrain fault signal
is per-wheel current (`cfg::kWheelStallAmps`).

See `docs/superpowers/specs/2026-08-04-rpi5-main-controller-design.md` for the
control split and the link watchdog.

---

## 2. GPIO map — ESP32-S3-DevKitC-1 **N16R8**

Extracted from the netlist. Mirrored in `include/pins.h`; that file is the code's
copy of this table.

### Wheel motors (index order FL, FR, RL, RR — matches `tb::Corner`)

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
| **Arm UART1 TX / RX** | **1 / 2** | to ESP-ARM GPIO44 / GPIO43 — see §5. Formerly the FL/FR encoder headers, which must stay unpopulated |

### Reserved and free

| Pins | Status |
|---|---|
| 26–37 | flash + octal PSRAM — **board correctly leaves 35/36/37 unconnected** |
| 19, 20 | native USB — unconnected |
| 0, 45, 46 | strapping — unconnected |
| 3, 14 | freed by the encoder removal — 3 is strapping (JTAG select), 14 is ADC2 (dead once WiFi is on) |
| **43, 44** | UART0 — free **only** in a native-USB-CDC build. `bench_ble` is not one (`CONFIG_ESP_CONSOLE_UART_NUM=0`), which is why the arm link uses 1/2 |

26 GPIO used, no duplicates, no reserved-pin conflicts. Verified programmatically.

---

## 3. BLD-120A driver interface

### Speed (SV)

The driver has a **native PWM speed input**: manual, Speed Command mode C —
*"speed can be adjusted by PWM control between 1KHz~10KHz, motor speed is
influenced by duty."* No DAC, op-amp or RC filter needed.

- `kSvPwmFreqHz = 5000` — **1–10 kHz is a hard spec range**, not a preference
- 1–10 kΩ series resistor between GPIO and SV (manual FAQ A). 1.5 kΩ in use.
- **SV is a high-impedance voltage input.** Diode test 2026-08-05 read open in
  both directions, unlike EN/BRK/F-R which show a diode drop. It does *not* load
  the source — the old "pot-wiper drags it down" claim, and the 2.59 V figure
  behind it, were the ESP32's weak DAC on the WROOM-32D bench rig, not the driver.

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
| **P-sv** | overload power limit, 30–120 W | **set to the motor's rated watts** — wrong value trips the red LED and the motor won't run |

P-sv is the *only* hardware overload protection on this drivetrain. It is not a
speed trim, despite what earlier bench notes claimed.

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
switch `tb::kControlViaMosfet` in `Bld120aMotor.h`, which derives all four
polarity constants and forces push-pull gate drive. Verified by `static_assert`.

**Device:** BC547 or 2N3904 (NPN, from existing stock). Base current
(3.3 − 0.7)/10 k = 260 µA against a 440 µA load — saturates hard.

> ⚠️ **BC547 is C-B-E; 2N3904 is E-B-C** (flat face toward you, legs down).
> Identical packages, mirrored pinouts. Pick one type and stay with it.

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

### Base ↔ ESP-ARM serial link

The base drives the arm with single characters over **UART1 on GPIO1/GPIO2**.

```
base GPIO1 (TX)  ────────►  arm GPIO44 (RX)
base GPIO2 (RX)  ◄────────  arm GPIO43 (TX)
base GND         ─────────  arm GND
```

**The crossing is the whole thing.** Straight-through ties TX to TX and RX to RX:
two outputs driving each other, two inputs floating, nothing transmitted either
way — and it measures perfectly on a continuity test.

Three constraints that are not obvious from either codebase:

1. **GPIO1/GPIO2 are the FL/FR encoder headers.** Leave them unpopulated. An
   AS5600's analog `OUT` is actively driven and would fight the UART's TX.
2. **The arm's `Serial` is UART0 on GPIO43/44, not native USB.** Its
   `platformio.ini` leaves `build_flags` empty, so `ARDUINO_USB_CDC_ON_BOOT`
   defaults to 0 and no CDC object is linked into the binary at all. Plugging
   into the DevKitC's *native USB* port gives a port that enumerates, stays
   silent, and ignores everything typed at it.
3. **The arm discards anything arriving close behind a command.** Its handler
   reads one character then flushes the rest of the buffer, so two bytes sent
   back to back lose the second. The base therefore paces its transmissions
   (`ARM_GAP_MS` in `sketch_pcb.cpp`) rather than writing directly.

Diagnosing a silent link: the arm prints a ~20-line banner at boot unconditionally,
so power-cycling it with the base monitor open tests the arm→base direction for
free. The base's status line carries an `arm-rx` byte counter, and the **Menu**
button runs an active probe.

### Encoders — removed 2026-08-13

Four AS5600 on a TCA9548A mux, never made to work: all four share address `0x36`
with no address pin, the board commons SDA/SCL, and isolating them needed eight
trace cuts at the encoder connectors. Resistance checks found two modules at
60 kΩ and 165 kΩ against 280 kΩ for the healthy pair, and the parasitic-power
signature pointed at a missing VCC connection in the loom.

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
2. **Supply rail.** The board feeds J8/J9/J12/J14/J17 from **3.3 V** (`3V Wheel`).
   ACS758 is ratiometric, so at 3.3 V a -050B gives 26.4 mV/A with a 1.65 V zero
   point — read directly by a 3.3 V ADS1115, no divider, no level shifting.
   Powering it at 5 V instead puts up to 5 V into a 3.3 V ADC input and needs a
   divider. **Confirm the rail before wiring.**
3. The sensor is *observation*, not protection. The real trip chain is the driver's
   P-sv overload trim, then the 10 A fuse, then software.

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

## 9. Open items

- [ ] **Minimum speed.** Break-away ≈0.78 V → ~413 mm/s at the wheel, already
      above the 300 mm/s target, and one 20 ms tick moves 8.3 mm against an
      8 mm centring tolerance. Likely a gearing/wheel-diameter problem, not a
      firmware one. The driver's PID speed loop may rescue it — **settle with the
      REF+ → 1 kΩ → SV pot sweep.**
- [ ] `max_lin_mm_s = 300` is wrong; actual is ~1400 mm/s at full command.
      Odometry currently under-estimates every distance by ~4.7×.
- [ ] `main.cpp` still passes one shared EN/BRK pair to all four motors.
- [ ] ACS758LCB-**050B** confirmed as the right part (§6); still to confirm the
      supply rail is 3.3 V and set ADS1115 ADDR→GND for `0x48`.
- [ ] Confirm the motor's rated watts, to set P-sv.

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
