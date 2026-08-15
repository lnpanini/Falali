# Gotchas — things that cost days, written down so they cost minutes

Every entry here was found on hardware, usually after hours of looking somewhere
else. They share a shape: **the fault and the correct behaviour produce identical
observations.** That is why they were expensive, and why a list of them is worth
more than another architecture diagram.

Read this before changing anything in `bench_ble/`, `arm/`, or `include/pins.h`.

## How to read this file

Entries are tagged, because they are not all equally certain and treating a good
guess as a fact is how the next person loses a day:

- **[VERIFIED]** — read in the source, or measured on hardware, and quoted here.
  You can build on it.
- **[OBSERVED]** — this happened on the robot and the log is quoted. What caused
  it may still be open.
- **[HYPOTHESIS]** — a model that fits the evidence and has not been tested
  against an alternative. It may be wrong. Where a hypothesis is load-bearing,
  the experiment that would confirm or kill it is written alongside.

The tags are the honest part. Several things below were confidently believed and
then turned out to be something else — a boot failure blamed on a jumper that was
never fitted, a dead link blamed on wiring that was correct. Both cost more than
the eventual fix.

---

## 1. A dead ToF sensor reads as "clear air"  **[VERIFIED]**

`arm/src/TofSensorArray.cpp` substitutes `TOF_OUT_OF_RANGE_MM` = **8191** when
`RangeStatus == 4`, and the air test is:

```cpp
return reading.distanceMm > thresholdMm;      // thresholdMm = 200
```

**8191 > 200.** So a sensor that is disconnected, failed, saturated, or simply
pointing at open space reports the exact condition that tells the arm it may stop
extending and start flipping.

Seen as: *"neither arm is extending"* during `grab`. `startAxisCycle` reads the
ToF before doing anything and, if air is already detected, **skips the extend
phase entirely**. On a bench with nothing overhead, that is every time.

It also makes the visible axis order look random — an axis whose ToF reads air
skips its extend, so which arm you *see* move varies with sensor state even
though the command order is fixed.

The arithmetic and the code path are verified. That this is what produced the
observed *"neither arm is extending"* is **[HYPOTHESIS]** — strongly supported
(the shortcut is unconditional and the bench had nothing overhead) but never
tested by covering the sensors and re-running. **That test takes ten seconds and
nobody has done it.**

**Not fixed.** It is upstream's call. If you touch it: the fix is to treat
`RangeStatus == 4` as *unknown*, not as a distance, and to refuse to act on
unknown.

---

## 2. Extend and retract check different limit switches  **[VERIFIED]**

`arm/src/main.cpp`:

```cpp
bool isXExtendBlocked(...)  { return states.xArmMaxPressed; }
bool isXRetractBlocked(...) { return states.xArmMinPressed ||
                                     states.limitX1Pressed || states.limitX2Pressed; }
```

| direction | switches | pins |
|---|---|---|
| X extend | `X_ARM_MAX` alone | **GPIO42** |
| X retract | `X_ARM_MIN`, `LIMIT_X1`, `LIMIT_X2` | GPIO2, GPIO11, GPIO48 |
| Y extend | `Y_ARM_MAX` alone | **GPIO1** |
| Y retract | `Y_ARM_MIN`, `LIMIT_Y1`, `LIMIT_Y2` | GPIO3, GPIO5, GPIO13 |

**Testing the limit switch by retracting proves nothing about extending.** They
are different switches on different GPIOs that happen to share a name. Retract
has two spare inputs to catch a failure; extend has none.

**The table is verified from source. Why extend does not stop on hardware is
[HYPOTHESIS] and remains open — see "Open issues" at the end of this file.**
The leading theory is that `X_ARM_MAX` (GPIO42) is not reading, since the switch
that demonstrably works during retract is one of three *different* inputs. That
has not been confirmed with a meter or the switch monitor.

If that theory is right, extend has no mechanical protection at all — what
remains is the ToF (see §1, which cannot be trusted to stop anything) and an 8 s
timeout at duty 120/255.

Diagnose with the arm's `m` command over its USB serial: a live switch monitor
that prints on every state change with the motors off.

---

## 3. The arm's command mailbox holds exactly one command  **[VERIFIED]**

`arm/src/main.cpp`'s receive callback overwrites `pendingWirelessCommand`
unconditionally, and `loop()` reads whatever is sitting in it. **Two commands
sent closer together than one arm loop collapse into the second** — the first
never happened, with no reply and no error.

Measured: a jog followed immediately by its own stop produced **30 sends and 2
replies**. The arm saw `mstop` and almost never `xret`, so the motor never
started and the jog looked dead while the base's console said "sent".

The base paces its sends for this reason (`ARM_GAP_MS` in `sketch_pcb.cpp`).
**Servo commands need far more room** — `moveServoPairSlow` ramps 1° per 20 ms
across 90° as a blocking loop, so the arm's `loop()` does not run for ~1.8 s;
hence `ARM_SERVO_GAP_MS` = 2200.

The same class of bug bit the UART link first, where the arm read one character
then flushed the rest of the buffer. Different mechanism, opposite victim,
identical symptom. **Do not remove the pacing because the transport changed.**

---

## 4. ESP-NOW hands you every frame on the channel  **[VERIFIED]**

`esp_now_register_recv_cb` fires for frames from devices you never peered with —
broadcast ESP-NOW reaches every ESP-NOW device on the channel. In a lab full of
ESP32 projects that is constant traffic; two other senders were logged here at
~15 frames/second.

The base filters on `info->src_addr` and counts the rest as `ign` on the status
line. **The filter is not cosmetic.** Before it existed, every stray frame ran
the reply parser, and since none of them equalled `"busy"` each one *cleared the
busy interlock* — so the arm could be 20 s into a 40 s workflow and the base
would consider it idle.

---

## 5. `ARM_ESP_MAC` in the arm's config is decorative  **[VERIFIED]**

`arm/src/HardwareConfig.h` defines it, and it is **never applied with
`esp_wifi_set_mac()`** — only printed at boot as a label. It is a note about what
someone believed the MAC to be, not something that makes it so.

The base checks its own MAC against `SELF_ESP_MAC` at boot and says so if it
differs. Do the same before trusting either constant. A wrong peer MAC gives
perfect silence with nothing wrong on screen.

---

## 6. `Serial` does not mean what you think in either firmware  **[VERIFIED]**

**`bench_ble/` (base):** Bluepad32 owns the console.
`CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=y`, and `sdkconfig.defaults` says why in its
own comment — *"Arduino Serial conflicts with the console"*. `Console` is
**output only** (no `available()`, no `read()`), `Serial` is never `begin()`-ed,
and stdin belongs to Bluepad32's REPL.

`handleConsole()` in `sketch_pcb.cpp` has therefore **never run.** Every keyboard
command in it is dead, and was dead for three sketches before anyone noticed —
because output worked, so nothing looked broken. **Operator controls go on
gamepad buttons.**

**`arm/` (arm):** `platformio.ini` leaves `build_flags` empty, so
`ARDUINO_USB_CDC_ON_BOOT` defaults to 0 and `Serial` resolves to UART0 on
GPIO43/44. Verified against the linked binary: `firmware.map` contains **zero**
`HWCDC`/`USBCDC` objects. Plugging into the native USB port gives a port that
enumerates, stays silent, and ignores you.

---

## 7. The VL53L0X library returns failed measurements as valid numbers  **[VERIFIED code, HYPOTHESIS cause]**

The vendored Pololu driver returns the raw range register and **never reads
`RANGE_STATUS`**. A measurement that failed on signal, sigma or phase comes back
looking exactly like a good one — `0`, `74`, `221`, `611`, `837` mm were all
logged with nothing above the sensor.

That the driver ignores `RANGE_STATUS` is verified in its source. That the
specific garbage values were *failed measurements* rather than something else —
bus corruption, a stale register read — is **[HYPOTHESIS]**. It fits (the values
are unstable and appear only with no target) and `setSignalRateLimit(0.50)` was
applied on that basis, but no one has read the status register to confirm.

If the hypothesis holds, debouncing cannot help: these are genuine measurements
that genuinely failed, and several land in a row, so the rejection has to happen
**at the device**.

---

## 8. Docking's band ceilings must stay equal  **[HYPOTHESIS — a model that fits]**

`cfg::makeCornerConfig()` sets `assert_max_mm` and `band_max_mm` to the **same**
value, and there is a comment saying not to split them. The reason is geometric:

The trolley edge is a vertical face, and the sensor's outermost rays graze it
from outside. A ray at the 12.5° cone edge reaches lateral offset `d` at slant
range `d / sin(12.5°)`, so the ceiling is really a *"how far past the edge am I"*
test — 130 mm of range is 28 mm of travel.

**This is a model, not a measurement.** It was built to explain readings of 138 →
219 → 296 mm logged with the sensor ~60 mm past the edge, and `60 / sin(12.5°)`
= 277 mm fits that closely. It has not been tested at a second standoff, which is
the obvious way to falsify it. The `kEdgeOverreachMm` constant derived from it
feeds the `max_lin_mm_s` calibration, so **if the model is wrong that constant is
wrong too** — treat both as provisional.

Entry is early by the same distance exit is late, and the bisection cancels it
**only while the two ceilings match.** A hysteresis gap of 130/160 puts a ~3 mm
bias straight into the midpoint. Hysteresis on the *floor* is free, because
readings climb at an edge crossing and never fall.

---

## 9. `pins.h` is netlist-derived, and the silkscreen disagrees with it  **[VERIFIED on hardware]**

The ToF connectors are labelled **4, 6, 7, 8** on the fabricated board and those
are FL, FR, RL, RR — but the GPIOs behind them are **4, 5, 6, 7**. Only the first
label matches its pin. Label "8" is GPIO7, and GPIO8 is the I²C SDA line, which
can never be an XSHUT: pulling it low during `Vl53l0xArray::begin()` takes the
whole bus down.

The current-sense connectors are worse — `J14`, `J17`, `J12`, `J9` map to FL, FR,
RL, RR, which is not numeric order. **Never infer a channel from a J-number.**

There is no netlist checked into this repo to verify either against; the KiCad
project under `KiCad/` is the authority.

---

## 10. GPIO traps on this board specifically  **[VERIFIED — datasheet + pins.h]**

| pin | trap |
|---|---|
| **GPIO48** | The DevKitC's onboard RGB LED **and** the base's FR motor ENABLE. Driving a WS2812 frame there fires an 800 kHz burst into a motor enable input — and HIGH is *asserted* through the inverting adapters. On the arm it is `LIMIT_X2`. |
| **GPIO1/2** | Routed to the FL/FR encoder headers. Free now, but an AS5600's `OUT` is actively driven, so anything plugged into those headers fights whatever else uses the net. |
| **GPIO14** | ADC2 — stops working the moment WiFi is enabled, and `bench_ble` now enables WiFi for ESP-NOW. Treat as digital-only. |
| **GPIO3** | Strapping pin (JTAG source select). |
| **GPIO43/44** | UART0. Free *only* in a native-USB-CDC build; `bench_ble` is not one. |

---

## 11. The board is 16 MB and was configured as 4  **[VERIFIED]**

`CONFIG_ESPTOOLPY_FLASHSIZE_4MB` on an N16R8 module, with a 1 MB app partition —
and **PlatformIO generates the partition table, not the sdkconfig.** Setting
`CONFIG_PARTITION_TABLE_CUSTOM` is silently ignored; the builder emits its own
from `board_build.partitions`.

That mismatch was harmless for months and then became a boot loop the moment
WiFi pushed the binary past 1 MB. `bench_ble/partitions.csv` now gives the app
3 MB and writes its offsets explicitly, because a blank offset is derived from
the preceding entry — so resizing anything shifts `nvs`, and shifting `nvs` wipes
the stored Bluetooth pairing keys.

---

## 12. Dead-reckoning integrates the *command*, not the wheels  **[VERIFIED]**

There are no encoders (see §13). `DeadReckonOdometry` multiplies commanded
velocity by `max_lin_mm_s`, so **a wheel that does not break away still accrues
travel in the estimate** and nothing can detect it.

Two consequences worth internalising:

- Speeds below break-away do not run slowly, they **do not run at all** — and the
  console cannot tell that from an ignored button press. 0.05 was tried; duty 205
  reached the wheels and the base never moved.
- `max_lin_mm_s` was estimated at 1155, 1231, 1384 and 1657 depending on method.
  The docking sequence was deliberately restructured so the **park point no
  longer depends on it** (see §14).

---

## 13. The encoders are gone and are not coming back  **[VERIFIED]**

Four AS5600 on a TCA9548A mux, never made to work: all share address `0x36` with
no address pin, the board commons SDA/SCL, and isolating them needed eight trace
cuts. Two modules measured 60 kΩ and 165 kΩ against 280 kΩ for the healthy pair.

Pin tables were **deleted** from `pins.h` rather than commented out — a table
describing hardware nobody drives is indistinguishable from one describing
hardware somebody does. `kMuxAddr` survives only because the bring-up scan
reports whether the mux is present.

**There is no wheel-speed feedback of any kind.** Per-wheel current
(`cfg::kWheelStallAmps`) is the only drivetrain fault signal, and it is still
uncalibrated under load.

---

## 14. Docking centres on *opposed* edges, and that is load-bearing  **[VERIFIED — design]**

The obvious method marks both edges with the leading sensor pair, which lands the
midpoint half a sensor span short — corrected by `front_offset_mm`. But that
offset is **real millimetres subtracted from an odometry-space position**, so it
is only correct while `max_lin_mm_s` is correct.

Marking the near edge with the **trailing** pair and the far edge with the
**leading** pair gives two marks symmetric about the platform centre. Their
midpoint *is* the centre, no offset needed, and both marks scale with the
odometry constant exactly as the current position does — so the park point is
independent of the least trustworthy number in the system.

`DockingConfig::centre_opposed_pairs` selects it. Do not "simplify" it back.

---

## 15. The docking sequence runs in a rotated frame  **[VERIFIED — design]**

The base **crabs in sideways**; manual driving stays nose-first. `DockFrame.h`
permutes the corner array and swaps the drive axes at the port boundary, so
`DockingStateMachine` never knew and its tests still cover it unchanged.

Why: nose-first put the 780 mm sensor span under an 860 mm trolley — ±40 mm of
margin. Sideways puts the 450 mm span on that axis instead: **±205 mm**.

The permutation and the command rotation **must always change together**. Rotate
one without the other and the machine drives along one axis while reading edges
from the perpendicular one — which does not fail loudly, it bisects nonsense.

---

## 16. Trolley wheels block one axis, so the base aligns on one only  **[VERIFIED — mechanical constraint]**

Once underneath, the trolley's own castors block travel along the robot's
fore-aft axis — the docking frame's *lateral* axis. `CenterY` would command a
motion the chassis physically cannot make and burn its timeout, so
`centre_lateral` is false and the arm's own sensors take that axis.

`Confirm` still demands all four corners. With fore-aft uncorrectable it is the
only thing left that catches a base parked badly on that axis.

---

# Open issues — handed over unfixed

These are known, reproducible, and **deliberately not fixed**. They are recorded
here rather than silently left because a handover that omits the open faults is
worse than no handover.

## A. X-axis extend does not stop at its limit switch **[OBSERVED — cause open]**

**Symptom, reproduced on hardware 2026-08-14:** during the `grab` clamping
sequence, the X axis extends past its mechanical stop. The limit switch is *not*
consulted in a way that halts it.

**What is established:**

- The code path is correct. `isAxisExtendBlocked()` is tested at the top of every
  `serviceAxisCycle()` call while extending, and again in `startAxisCycle()`
  before the motor starts. The ToF check sits in the `else` branch, so **a closed
  limit switch takes priority over any ToF state** — a covered ToF cannot mask it.
- Extend consults `X_ARM_MAX` (GPIO42) **and nothing else**. Retract consults
  three different inputs (§2).
- The limit switch demonstrably works during manual retract — which exercises
  `X_ARM_MIN`/`LIMIT_X1`/`LIMIT_X2`, none of which extend looks at.

**What is not established — these are hypotheses, in rough order of likelihood:**

1. `X_ARM_MAX` on GPIO42 is not wired, or is wired to a different input. Fits
   every observation, but **no meter has been put on that pin.**
2. There is no physical max-travel switch on the X axis at all, and the mechanical
   stop is purely a hard stop. Would mean the firmware is asking for a signal the
   machine does not produce.
3. GPIO42 is compromised as an input. It is in the ESP32-S3 JTAG block
   (MTCK/MTDO/MTDI/MTMS on 39–42) and GPIO3 — a JTAG-source strapping pin — is
   also in use as `Y_ARM_MIN`. **Speculative. No evidence beyond the pin numbers.**

**The experiment that settles it**, ~30 seconds: connect the arm over USB
(`cd arm && pio device monitor -e esp32s3`), send `m` for the live switch monitor
— motors off, prints on every state change — and press the X max stop by hand.

- `X Arm Max` changes → the input works; hypotheses 1 and 3 die, and the fault is
  that the switch is not reached at the position the mechanism stops at.
- `Limit X1` or `X2` changes instead → it is wired to a retract input, and the fix
  is one line in `isXExtendBlocked()`.
- Nothing changes → hypothesis 1 or 2. Meter from the switch to the pin.

**Why it matters more than it looks:** extend has no redundancy. If that switch
is absent, the only things stopping the motor are the ToF — which reports 8191 mm
as "clear air" when it fails (§1), so it cannot be relied on to stop anything —
and an 8-second timeout at duty 120/255. That is roughly 47 % into a hard stop
for eight seconds.

**Do not fix this by guessing which switch to add to `isXExtendBlocked()`.**
Picking wrong makes extend halt at the *retract* end, which will look like a
different bug entirely.

## B. `max_lin_mm_s` is calibrated to about ±6 % **[HYPOTHESIS-dependent]**

Estimates from four methods spanned 1155–1657 mm/s. The value in `config.h` rests
partly on the cone-geometry model in §8, which has not been independently tested.

Largely defused: docking was restructured so the park point does not depend on it
(§14). It still affects timeouts and a ~2.7 mm lag term. Worth re-measuring
properly — a timed run over a long straight, from a rolling start — before anyone
relies on the odometry for anything new.

## C. `kWheelStallAmps` has never been calibrated under load

Set to 10 A from bench measurements taken with the wheels off the ground. It is
the **only** drivetrain fault signal the base has (§13), and it has never seen a
loaded trolley. A current sensor also cannot distinguish a stall from a heavy
load, so treat it as protection against something catastrophic, not as a limit.
