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

## 9. Reference designators are not GPIO numbers  **[VERIFIED — netlist, 2026-08-16]**

**This entry was rewritten.** The original claimed the silkscreen contradicted
`pins.h`, and it was itself partly wrong. What follows was extracted from
`KiCad/Wheel Drive PCB/Wheel Drive PCB.net` directly. Full tables in
`docs/hardware/boards.md`.

**The reference designators do not track the GPIOs.** The ToF connectors are
`TOF5`–`TOF8` and the XSHUT lines behind them are GPIO **4, 5, 6, 7**. No
arithmetic relates the two, and there is no `TOF4`. GPIO8 is the I²C SDA line and
can never be an XSHUT — pulling it low during `Vl53l0xArray::begin()` takes the
whole bus down, ToF and IMU and ADS together.

**But the functional silkscreen is trustworthy, and it is what you should read.**
Every pin on this board is labelled with its function: the pin beside each ToF
connector's XSHUT reads `XSHUT FL` / `XSHUT FR` / `XSHUT RL` / `XSHUT RR`, and
the current-sense connectors are labelled `A0`–`A3`. Corner identity on the
board is unambiguous. Never infer a corner from a **J-number**; always read the
**label**.

### The correction that matters

The original entry said the current-sense connectors map `J14, J17, J12, J9` →
FL, FR, RL, RR — "which is not numeric order." **The netlist says otherwise:**

| Connector | J9 | J12 | J14 | J17 |
|---|---|---|---|---|
| ADS channel | A0 | A1 | A2 | A3 |
| Schematic name | **FL** | **FR** | **RL** | **RR** |

They *are* in numeric order. The same wrong table is repeated in the comments in
`include/pins.h`. **The code is unaffected** — `kCurrentAdsChannel = {0,1,2,3}`
for FL..RR matches the schematic — but the explanatory comment above it, and the
lesson this gotcha used to teach, were both wrong.

`pcb_identify.cpp` is cited in `pins.h` as the empirical authority for that
mapping. Note that it identifies a corner by watching an encoder **and** a
current channel move together, and half that procedure died with the encoders
(§13). It can no longer self-verify a corner.

**There is a netlist in this repo** — `KiCad/Wheel Drive PCB/Wheel Drive PCB.net`,
and one for each of the other two boards. The fabricated `.kicad_pcb` carries an
identical net set, so layout and schematic agree. Check against it rather than
re-deriving from a rendered schematic.

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
(`cfg::kWheelStallAmps`) was the only drivetrain fault signal, and as of
2026-08-16 it turns out not to be connected either — see Open Issue C.

> **[ADDED 2026-08-16] The motors have hall sensors. Nobody is using them.**
>
> The motor datasheet gives the cable as **`AWG20 × 3 + AWG26 × 5`** — three
> phase wires and five hall wires (V+, GND, HA, HB, HC). The BLD-120A just does
> not bring them out; it has no FG or tacho terminal, which is where "no speed
> feedback" came from.
>
> **If speed feedback is ever wanted back, tap the halls — do not revive the
> AS5600s.** One hall line per motor into a spare GPIO gives 3 pulses per motor
> revolution = 45 per wheel revolution at 15:1. That is plenty for stall
> detection and speed estimation, it needs no I²C, and it sidesteps both
> board-level faults that killed the encoders.
>
> **[HYPOTHESIS — wire count is from the datasheet; nobody has scoped them.]**
> *Experiment: 24 V applied, turn a wheel by hand, scope any AWG26 conductor
> against motor ground. A 3-per-rev square wave confirms it.*

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

## 17. The Arm PCB's servo headers are not what drives the servos  **[VERIFIED — netlist + source, 2026-08-16]**

The Arm Subsystem PCB has four servo headers, `S1`–`S4`, wired to **GPIO10, 47,
4 and 14**. They are labelled `Servo X1`, `Servo X2`, `Servo Y1`, `Servo Y2` in
the schematic. Everything about the board says "plug your servos in here."

**The firmware never touches those pins.** `arm/src/MotorController.h` holds an
`Adafruit_PWMServoDriver` and every servo movement goes through it — a **PCA9685
at `0x40` over I²C**, channels 0–3. `grep -rn "PCA9685\|setPWM" arm/src/` finds
the breakout; nothing in `arm/src/` mentions GPIO 10, 47, 4 or 14.

And **there is no PCA9685 connector on the board.** The only I²C access points
are the four ToF headers.

**Confirmed on the robot 2026-08-16 [OBSERVED — Bryan]:** a PCA9685 is fitted,
and it is plugged into **one of the ToF headers** to reach I²C. The servos go on
the PCA9685's own outputs. `S1`–`S4` are dead copper.

Two consequences of that arrangement worth knowing before you touch it:

- **One ToF header is doing double duty.** The arm still needs all four ToF
  sensors on the bus, so whichever sensor lost its header is daisy-chained
  through the PCA9685 breakout. That makes the arm's I²C a hand-built chain, not
  the star the board draws — check it physically before believing any bus map.
- The PCA9685 drives the **four flipper servos**, *not* the wormgear extension
  motors. The wormgear motors go through the two BTS7960 modules on direct GPIO
  (J1/J2). This is easy to misremember backwards. **[VERIFIED — `setPWM()` is
  called only from `setServoPulse()`, channels 0–3.]**

Why this is the expensive kind of mistake: a servo plugged into the header the
board provides, on a board whose every other connector works, will simply never
move. There is no error, no log line, and the obvious conclusion is a dead servo.

**Two ways of closing the mismatch.** Respinning the board with a PCA9685 header
is the intuitive one; rewriting `MotorController` to use four LEDC channels on
the pins already routed needs no new hardware and frees the I²C bus. Full detail
in `docs/hardware/boards.md` §3.

---

## 18. The big red mushroom button energises, it does not stop  **[VERIFIED — Bryan, 2026-08-16]**

It is not an E-stop. It is an **enable**, on the 12 V arm rail.

> **Pressed = arm rail ON. Released (twist-and-pull) = arm rail OFF.**

Under stress, everyone slaps a mushroom button. On this machine that **powers the
arm up.** It also does not switch the 24 V motor rail at all, so even used
correctly it does not stop the wheels — only firmware does.

The fitted part is a **1NO 1NC DPST** latching mushroom switch, so the
normally-closed contact is physically present and **currently unused**. The rail
is on the normally-open contact. Moved to NC the behaviour would invert to what
most people expect, and nothing else in the system would change.

It gates the **input** side of the 12 V buck, which is what makes the inrush
staggering in `docs/hardware/startup.md` §3 work.

As it stands it is a power switch wearing an emergency stop's hat. See
`docs/hardware/startup.md` §1.

---

## 19. The driver has a third trim that ramps every command  **[VERIFIED — spec sheet, 2026-08-16]**

Every note in this project listed **two** BLD-120A adjusters, RV and P-sv. There
are **three**. The missing one is **ACC/DEC**, and the datasheet is blunt about
what it does:

> *"This potentiometer can be used for adjusting acceleration and deceleration
> time directly … The range can be set is: 0.3s–15s."*

**The positions of all four drivers are unrecorded.** At anything above the short
end of that range the driver ramps every speed change over seconds — and the
firmware has no idea:

- `DeadReckonOdometry` integrates the **commanded** velocity (§12) and assumes
  the wheel follows immediately. Every start and stop then over-estimates
  distance travelled by the ramp.
- Docking issues short corrective moves. A move shorter than the ramp **never
  reaches its commanded speed at all.**
- It would look exactly like stiction — a wheel that "ignores" small commands.
  Which is precisely the symptom §12 records.

This has the shape everything else in this file has: **the fault and the correct
behaviour produce identical observations.** A sluggish wheel looks like friction,
looks like a bad calibration constant, looks like a firmware bug. It could also
be a dial someone turned once.

*Reading the four positions establishes whether this is a factor at all.
Changing them and re-running a dock would show whether it accounts for the
low-speed behaviour in §12.*

Two smaller things from the same document:

- **P-sv is marked in amps on some units and watts on others** (1.6–8 A vs
  30–120 W). Same setting. Ours is **5 A ≡ 120 W at 24 V**. Too high is a fault
  as well as too low — the FAQ says so explicitly.
- **The English manual's feature list is boilerplate and contradicts the rest of
  the document.** It advertises *"Speed signal output"* and *"Abnormal alarm
  signal output"*; the terminal table a few pages later lists only
  `SV COM F/R EN BRK`. There is no FG and no ALM pin — §13 and `pins.h` were
  right, and the feature bullet is the thing that is wrong.

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

**Hypothesis 1 is now dead. [VERIFIED — Arm PCB netlist, 2026-08-16]**

The Arm Subsystem PCB was copied into this repo on 2026-08-16 and its netlist
read. It says:

```
/X Arm Max : SW3.1[1]  U1.39[GPIO42/MTMS]
```

`SW3` is the header named **"X Limit Max"**, pin 1 goes to **GPIO42**, pin 2 goes
to ground. `LimitSwitches.h` sets `INPUT_PULLUP` on GPIO42 and treats LOW as
pressed. **The board and the firmware agree exactly.** The old leading
hypothesis — "`X_ARM_MAX` is not wired, or is wired to a different input" — is
ruled out at the PCB level. All eight limit-switch nets match `LimitSwitches.h`.

**What is not established — remaining hypotheses, re-ranked:**

1. **No switch is physically fitted at SW3**, or one is fitted but the mechanism
   reaches its hard stop before closing it. Nothing electrical would reveal this,
   and it is now the most likely explanation.
2. **A loom fault between the SW3 header and the switch** — open wire, bad crimp,
   connector one position off. Also invisible to the netlist.
3. GPIO42 is compromised as an input. It is in the ESP32-S3 JTAG block
   (MTCK/MTDO/MTDI/MTMS on 39–42) and GPIO3 — a JTAG-source strapping pin — is
   also in use as `Y_ARM_MIN`. **Now less likely, not more:** the *base* board
   drives GPIO42 as `FL SV` and it works. Still speculative.

**The experiment that settles it**, ~30 seconds: connect the arm over USB
(`cd arm && pio device monitor -e esp32s3`), send `m` for the live switch monitor
— motors off, prints on every state change — and press the X max stop by hand.

- `X Arm Max` changes → the input works end to end; hypotheses 2 and 3 both die,
  and the fault is that the switch is not reached at the position the mechanism
  actually stops at. Mechanical fix.
- `Limit X1` or `X2` changes instead → the loom is one connector off and the
  switch is landing on a retract input (hypothesis 2). Fix the loom — **do not**
  "fix" it by adding that input to `isXExtendBlocked()`, which would make extend
  halt at the retract end and look like a different bug entirely.
- Nothing changes → hypothesis 1 or 2. Meter from the switch contacts to the
  SW3 header, then from the header to pin GPIO42.

Since the netlist rules out a PCB fault, **start at the mechanism, not the
electronics.** Look for the switch before you look for the signal.

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

> **[NARROWED 2026-08-16 — motor datasheet]** The gearing gives a hard ceiling
> that no calibration method can exceed:
>
> ```
> 3000 RPM ÷ 15  = 200 RPM at the wheel
> π × 152 mm     = 477.5 mm circumference
> 200/60 × 477.5 = 1592 mm/s
> ```
>
> **This rests on the 15:1 ratio**, which is inherited from earlier project notes
> rather than read off the gearbox, and the purchase listing groups this motor as
> "ratio 10–18". At 18:1 the ceiling would be 1326 mm/s and would rule out 1384
> as well; at 10:1 it would be 2388 mm/s and rule out nothing. Confirming the
> ratio from the gearbox label would firm all of this up.
>
> **The 1657 mm/s estimate is 104 % of that and is rejected outright.** 1155,
> 1231 and 1384 all survive at 73 %, 77 % and 87 % of ceiling respectively. This
> does not confirm the configured 1231, but it caps the range from above with a
> number that owes nothing to the cone model in §8 — the first independent
> constraint this quantity has had. Datasheet in
> `docs/hardware/img/motor-80-flange-spec-table.jpeg`.

Largely defused: docking was restructured so the park point does not depend on it
(§14). It still affects timeouts and a ~2.7 mm lag term. Worth re-measuring
properly — a timed run over a long straight, from a rolling start — before anyone
relies on the odometry for anything new.

## C. The current sensors are not connected, so there is no fault signal at all **[OBSERVED — Bryan, 2026-08-16]**

**This entry used to say `kWheelStallAmps` was merely uncalibrated. It is worse
than that.**

The four ACS758 are physically installed, inline with the **negative side of the
motor driver supply** — but their outputs are **not wired to J9/J12/J14/J17**.
They conduct current and report to nobody. `StallDetector` reads an ADS1115 that
sees nothing.

Combined with §13 (no encoders) and the absence of an ALM terminal, **the base
has no drivetrain fault detection whatsoever.** What remains is the driver's P-sv
overload trim and the pack fuse.

Connecting them is four wires, one per sensor, from each `VIOUT` to pin 4 of its
connector; 3.3 V and ground are already on pins 1 and 2. Low-side sensing is
electrically fine — the ACS758's conduction path is galvanically isolated from
its signal side. Corner mapping: **J9=FL=A0, J12=FR=A1, J14=RL=A2, J17=RR=A3**
(`docs/hardware/boards.md` §1).

The original caveat would still apply afterwards. `kWheelStallAmps = 10 A` came
from bench measurements with the wheels off the ground and has never seen a
loaded trolley. A current sensor also cannot distinguish a stall from a heavy
load, so it is protection against something catastrophic rather than a limit.
