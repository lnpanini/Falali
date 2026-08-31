# The three PCBs — connector reference

Every table here was extracted from the KiCad netlists, not from a rendered
schematic and not from `pins.h`. Where firmware disagrees with the netlist, the
disagreement is called out rather than smoothed over.

| Board | Size | KiCad project | Job |
|---|---|---|---|
| **Wheel Drive PCB** | 130 × 105 mm | `KiCad/Wheel Drive PCB/` | base ESP + 4 motor drivers + all base sensors |
| **Driver PCB** ×4 | 42.8 × 33.8 mm | `KiCad/Driver PCB/` | 3.3 V → open-collector adapter, one per motor |
| **Arm Subsystem PCB** | 110 × 110 mm | `KiCad/Arm Subsystem PCB/` | arm ESP + 2 H-bridges + 4 servos + 8 limit switches |

Schematic / layout / 3D-view renders for every board (including the historical
PCA9548A breakout) are indexed in [`KiCad/README.md`](../../KiCad/README.md#board-renders).

The Arm Subsystem PCB is **by Kai Xiang and Heng Li, EPD Batch of 2028**, as is
the `arm/` firmware.

---

## How to read a connector on these boards

The silkscreen is better than the older docs imply. **Every pin is functionally
labelled** — `FL SV`, `XSHUT FL`, `A0`, `Encoder FL`, `3V3`, `GND`. You never
have to infer a corner from a J-number; read the label next to the pin.

Two things the silkscreen does *not* tell you, which is what the tables below
are for:

1. **Which GPIO** is behind a labelled pin.
2. **Pin 1.** It is the square pad. On a 1×N header every other pad is round.

---

## 1. Wheel Drive PCB

**U6** is an ESP32-S3-DevKitC-1 **N16R8**. **Socket it** — 2× 1×22 female
header — do not solder the DevKit down. It is the most likely part to need
swapping and the most expensive to destroy getting off.

### J4 / J5 / J6 / J7 — BLDC driver outputs

One per wheel. These go to the **Driver PCB**, never straight to a BLD-120A.

| Pin | Signal | FL (J4) | FR (J5) | RL (J6) | RR (J7) |
|---|---|---|---|---|---|
| 1 | BRK | GPIO39 | GPIO38 | GPIO15 | GPIO13 |
| 2 | EN | GPIO40 | GPIO48 | GPIO16 | GPIO12 |
| 3 | F/R | GPIO41 | GPIO47 | GPIO17 | GPIO11 |
| 4 | GND / COM | — | — | — | — |
| 5 | SV | GPIO42 | GPIO21 | GPIO18 | GPIO10 |

**EN and BRK are per-wheel, not ganged.** Each wheel can be shut down
independently. `hardware-architecture.md` §9 still lists "main.cpp passes one
shared EN/BRK pair to all four motors" as an open item — check that before
trusting per-wheel shutdown.

> ⚠️ **GPIO48 is also the DevKitC's onboard RGB LED.** Driving a WS2812 frame on
> it fires an 800 kHz burst into the FR motor's ENABLE — and because the adapter
> inverts, HIGH is *asserted*. Never use the onboard LED on this board.

### TOF5 / TOF6 / TOF7 / TOF8 — VL53L0X time-of-flight

| Pin | Signal | FL (TOF5) | FR (TOF6) | RL (TOF7) | RR (TOF8) |
|---|---|---|---|---|---|
| 1 | VIN | 3V3 | 3V3 | 3V3 | 3V3 |
| 2 | GND | — | — | — | — |
| 3 | SCL | GPIO9 | ← shared | ← | ← |
| 4 | SDA | GPIO8 | ← shared | ← | ← |
| 5 | GPIO (module interrupt) | **not connected** | | | |
| 6 | **XSHUT** | GPIO4 | GPIO5 | GPIO6 | GPIO7 |

All four sensors are `0x29` from the factory. They are brought up one at a time
via XSHUT and re-addressed — that is what `lib/hal_esp32/Vl53l0xArray.h` does. No
mux involved.

The breakout **must** break out XSHUT. A VL53L0X module without an XSHUT pin
cannot be used here.

> **Correcting `gotchas.md` §9.** That entry warns the ToF connectors are
> "labelled 4, 6, 7, 8" and that only the first label matches its pin. The
> reference designators are **TOF5–TOF8**, and the functional silkscreen beside
> pin 6 reads `XSHUT FL` / `XSHUT FR` / `XSHUT RL` / `XSHUT RR`. Corner identity
> on the board is unambiguous. **[VERIFIED — netlist + silkscreen extraction]**

### J8 — ADS1115 current-sense ADC

| Pin | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
|---|---|---|---|---|---|---|---|---|---|---|
| Signal | 3V3 | GND | SCL | SDA | **GND** | NC | A0 | A1 | A2 | A3 |

Pin 5 lands on the module's **ADDR** pin and the board ties it to ground, so the
**ADS1115 address is `0x48`, fixed in copper.** `hardware-architecture.md` §9
lists "set ADS1115 ADDR→GND for 0x48" as an open to-do — it is already done, and
there is no jumper to get wrong. **[VERIFIED — netlist]**

Pin 6 is the module's ALRT output, deliberately unconnected.

### J9 / J12 / J14 / J17 — ACS758 current sensors

| Pin | Signal | J9 | J12 | J14 | J17 |
|---|---|---|---|---|---|
| 1 | VCC | **3V3** | 3V3 | 3V3 | 3V3 |
| 2 | GND | — | — | — | — |
| 3 | — | not connected | | | |
| 4 | OUT | **A0** | **A1** | **A2** | **A3** |
| | schematic name | **FL** | **FR** | **RL** | **RR** |

Two corrections here, both worth making before someone re-derives them:

> **The sensors run on 3.3 V.** `hardware-architecture.md` §6 leaves "confirm the
> rail before wiring" open — pin 1 of all four connectors is on the `/3V Wheel`
> net. The ACS758 is ratiometric, so at 3.3 V a `-050B` gives 26.4 mV/A around a
> 1.65 V zero, read directly by the 3.3 V ADS1115. **No divider, no level
> shifting.** Powering it at 5 V instead would put up to 5 V into a 3.3 V ADC
> input. **[VERIFIED — netlist]**

> **`pins.h`'s connector table is inverted, and `gotchas.md` §9 draws the wrong
> lesson from it.** `pins.h` comments claim `J14, J17, J12, J9` = FL, FR, RL, RR
> and §9 concludes "never infer a channel from a J-number ... not numeric order."
> The netlist says the connectors *are* in order: **J9=FL=A0, J12=FR=A1,
> J14=RL=A2, J17=RR=A3.** The **code is correct** — `kCurrentAdsChannel =
> {0,1,2,3}` for FL..RR matches the schematic. Only the explanatory comment is
> wrong. **[VERIFIED — netlist]**
>
> Note that `pcb_identify.cpp`, cited as the empirical authority, identifies a
> corner by watching an encoder *and* a current channel move together. Half that
> procedure died with the encoders, so the tool can no longer self-verify a
> corner. Read the `A0`–`A3` silkscreen instead.

### J19 — BNO085 IMU

| Pin | 1 | 2 | 3 | 4 | 5–10 |
|---|---|---|---|---|---|
| Signal | 3V3 | GND | SCL | SDA | not connected |

> ⚠️ **Check the module pinout before plugging this in.** The board provides a
> generic 10-pin header with only four pins live, in the order 3V3 · GND · SCL ·
> SDA. Several common BNO085 breakouts (the Adafruit BNO08x among them) run
> **VIN · 3Vo · GND · …**, where the module's pin 2 is a regulator *output*.
> Landing that on the board's GND pin shorts the module's 3.3 V regulator to
> ground. **[HYPOTHESIS — the netlist records only "Pin_2", so the intended
> module is not captured anywhere in the project.]** *Check: compare the module's
> own silkscreen against the four labels on J19 before it goes in. If they do not
> line up 1:1, wire it with flying leads, not by seating it on the header.*

### J1 / J11 / J15 / J18 and J2 / J10 / J13 / J16 — encoders (dead)

The AS5600 encoders were abandoned 2026-08-13. The headers remain.

| 1×04 (J1, J11, J15, J18) | 1 | 2 | 3 | 4 |
|---|---|---|---|---|
| Signal | **not connected** | SDA | SCL | GND |

| 1×03 (J2, J10, J13, J16) | 1 | 2 | 3 |
|---|---|---|---|
| Signal | 3V3 | OUT | GND |
| FL (J2) → GPIO1 · FR (J10) → GPIO2 · RL (J13) → GPIO3 · RR (J16) → GPIO14 | | | |

> **Why they could never work, from the netlist.** Two independent board-level
> faults:
>
> 1. **Pin 1 of every 1×04 connector is unconnected — the I²C-side header has no
>    VCC.** `hardware-architecture.md` §5 blames "a missing VCC connection in the
>    loom". It is in the PCB. That is exactly the parasitic-power signature that
>    was observed. **[VERIFIED — netlist]**
> 2. All four AS5600 share `0x36` with no address pin, and the board **commons
>    SDA/SCL across all four**. As built they cannot be addressed individually;
>    isolating them needs eight trace cuts.
>
> Bryan's later attempt used a PCA9548-type I²C mux externally; it overheated and
> failed with more than two sensors attached. **[OBSERVED — Bryan]**
>
> If anything reclaims GPIO1/2/3/14, remember they are still routed to these
> headers. An AS5600's `OUT` is an actively driven analog output and will fight
> whatever else is on the net. **Unplug the encoder looms rather than trusting
> the pins are free.**

### J3 — Common Ground

| Pin | 1 | 2 |
|---|---|---|
| Signal | board ground | **not connected** |

Ground bond only — **not** a power input. This is the single most important
connection on the board; see `startup.md` §2.

### Pins the board deliberately leaves alone

`GPIO0 · 19 · 20 · 26–37 · 43 · 44 · 45 · 46`, plus the DevKit's `5V` and
`CHIP_PU` pads. **[VERIFIED — netlist]** 26–37 are flash and octal PSRAM;
19/20 are native USB; 0/45/46 are strapping.

Genuinely free: **GPIO43 and GPIO44** — but only in a build whose console is
native USB-CDC. `bench_ble` is not one, so the IDF console sits on UART0 =
GPIO43/44.

---

## 2. Driver PCB — the 3.3 V → 5 V adapter

**You need four.** One channel-set per board.

This is the transistor adapter from `hardware-architecture.md` §4, fabricated.
Reconstructed from `DriverPCB.dsn`:

```
J1 "ESP"                                              J2 "BLD-120A"
 pin 1 BRK ──10k(R1)──┬── B  Q1 ── C ────────────────► pin 1 BRK
                   100k(R6)      E ──┐
 pin 2 EN  ──10k(R2)──┬── B  Q2 ── C ─┼──────────────► pin 2 EN
                   100k(R5)      E ──┤
 pin 3 F/R ──10k(R3)──┬── B  Q3 ── C ─┼──────────────► pin 3 F/R
                   100k(R4)      E ──┤
                        └────────────┤
 pin 4 COM ─────────────────────────┴───────────────► pin 4 COM
 pin 5 SV  ──────────1.5k(R7)───────────────────────► pin 5 SV
```

| Ref | Value | Job |
|---|---|---|
| Q1, Q2, Q3 | **2N3904** (TO-92) | open-collector switch, one per control line |
| R1, R2, R3 | 10 kΩ | base series |
| R4, R5, R6 | 100 kΩ | **base-to-COM pulldown — critical** |
| R7 | 1.5 kΩ | SV series (manual asks for 1–10 kΩ) |

**Why the 100 kΩ matters:** it holds the transistor off while the GPIO is
high-impedance — at boot, in reset, or unplugged. Without it the driver inputs
float and the motor state at power-up is undefined. This is what makes the
power-up behaviour in `startup.md` §1 safe.

**The transistor inverts every control line.** Firmware handles this with the
single switch `fal::kControlViaMosfet` in `Bld120aMotor.h`, which derives all four
polarity constants and is checked by `static_assert`. Do not invert a second time
in application code.

> ⚠️ **Do not substitute a BC547.** The footprint is `TO-92_Inline_Wide_**EBC**`
> and the symbol is a 2N3904 (**E-B-C**). **BC547 is C-B-E** — same package,
> mirrored pinout. Dropped into these holes unrotated it swaps collector and
> emitter and reproduces exactly the fault that `hardware-architecture.md` §4
> records as costing an evening. `hardware-architecture.md` §4 currently says
> "BC547 **or** 2N3904"; on this PCB that is only true if you rotate the BC547
> 180°. **[VERIFIED — footprint + symbol]**

### Diagnosing a dead channel

From the failure that cost that evening — these are the two measurements that
localise it in under a minute:

| Measurement | Healthy | Faulty (what was seen) |
|---|---|---|
| base → COM, GPIO driven high | **~0.7 V** (junction clamps) | 1.678 V — junction not conducting |
| collector → COM, enabled vs disabled | large change | 1.069 → 0.945 V — barely moves |

A base that will not clamp at ~0.7 V means the B–E path to COM is broken. A
collector that does not move between states means the transistor is not
switching. Currents are microamps throughout — you cannot damage anything by
probing.

### Intermittent channels — clean between the traces

**[OBSERVED — Bryan]** A channel that works, then does not, then does again is
usually **flux residue** between the traces rather than a component fault. Old
no-clean flux absorbs moisture and oxidises, and the leakage across a 200 µm gap
is enough to matter at the microamp currents this board runs at.

Isopropyl alcohol and a stiff brush between the traces, dried thoroughly. Do this
**before** replacing parts — it is the cheaper hypothesis and it is usually
right.

---

## 3. Arm Subsystem PCB

**U1** is a second ESP32-S3-DevKitC-1 N16R8. Board is 110 × 110 mm, 2-layer.

Firmware: `arm/` (vendored — see `arm/LOCAL-CHANGES.md`).

### J1 / J2 — BTS7960 H-bridge, X and Y extension motors

| Pin | Signal | X (J1) | Y (J2) |
|---|---|---|---|
| 1 | +5V | — | — |
| 2 | GND | — | — |
| 3 | L_EN | GPIO7 | GPIO41 |
| 4 | R_EN | GPIO15 | GPIO40 |
| 5 | LPWM | GPIO16 | GPIO39 |
| 6 | RPWM | GPIO17 | GPIO38 |

Matches `arm/src/HardwareConfig.h` exactly. **[VERIFIED]**

The 12 V motor supply goes to the **BTS7960's own B+/B− screw terminals**, not
through this PCB. This header carries logic only.

### S1 / S2 / S3 / S4 — servo headers

| Pin | 1 | 2 | 3 |
|---|---|---|---|
| Signal | signal | +5V | GND |

| Header | Schematic name | Signal pin |
|---|---|---|
| S1 | Servo X1 | GPIO10 |
| S2 | Servo X2 | GPIO47 |
| S3 | Servo Y1 | GPIO4 |
| S4 | Servo Y2 | GPIO14 |

> ⚠️ **These four headers are not used by the shipped firmware.**
> `arm/src/MotorController.h` drives all four servos through an
> `Adafruit_PWMServoDriver` — a **PCA9685 at `0x40` over I²C**, channels 0–3.
> GPIO10, 47, 4 and 14 are never referenced anywhere in `arm/src/`.
>
> There is **no PCA9685 footprint or connector on this board.** The only I²C
> access points are the four ToF headers.
>
> **[VERIFIED — netlist has no PCA9685; `grep -rn "PCA9685\|setPWM" arm/src/`
> shows the firmware talks only to the breakout.]**
>
> **As actually built [OBSERVED — Bryan, 2026-08-16]:** a PCA9685 is fitted and
> **plugged into one of the ToF headers** to reach I²C, with the servos on its own
> outputs. `S1`–`S4` are dead copper. The displaced ToF sensor is chained through
> the breakout, so **the arm's I²C is a hand-built daisy chain, not the star this
> board draws.** Trace it physically before trusting any bus diagram — including
> the one in `hardware-architecture.md`.
>
> The mismatch is between the board and the firmware, not a defect in either.
> Two ways of closing it: respin the board with a PCA9685 header, or rewrite
> `MotorController` to use four LEDC channels on the pins already routed. The
> second needs no new hardware, frees the I²C bus, and returns the fourth ToF
> sensor to its own header.

### SW1–SW8 — limit switches

Two-pin headers: **pin 1 = signal, pin 2 = GND.** Plain switch-to-ground; there
are no pull-up resistors on the board, and the firmware sets `INPUT_PULLUP` on
all eight. **Pressed reads LOW.**

| Header | Schematic name | Firmware name | GPIO |
|---|---|---|---|
| SW3 | X Limit Max | `X_ARM_MAX` | **GPIO42** |
| SW7 | X Limit Min | `X_ARM_MIN` | GPIO2 |
| SW8 | Y Limit Max | `Y_ARM_MAX` | GPIO1 |
| SW6 | Y Limit Min | `Y_ARM_MIN` | GPIO3 |
| SW1 | Limit Switch X1 | `LIMIT_X1` | GPIO11 |
| SW4 | Limit Switch X2 | `LIMIT_X2` | GPIO48 |
| SW2 | Limit Switch Y1 | `LIMIT_Y1` | GPIO5 |
| SW5 | Limit Switch Y2 | `LIMIT_Y2` | GPIO13 |

All eight match `arm/src/LimitSwitches.h` exactly. **[VERIFIED]**

> ### This kills one hypothesis in `gotchas.md` Open Issue A
>
> Open Issue A is "X-axis extend does not stop at its limit switch", and its
> leading hypothesis was **"`X_ARM_MAX` on GPIO42 is not wired, or is wired to a
> different input."**
>
> **The netlist rules that out.** SW3 "X Limit Max" connects to GPIO42 and to
> ground, the firmware reads GPIO42 with `INPUT_PULLUP`, and pressed = LOW. The
> PCB is correct and the firmware matches it. **[VERIFIED]**
>
> What survives, re-ranked:
>
> 1. **No switch is physically fitted at SW3**, or it is fitted but the mechanism
>    stops before reaching it. Nothing in the electrical design would show this.
> 2. **A loom fault** between the SW3 header and the switch — open wire, bad
>    crimp, connector off by one.
> 3. GPIO42 compromised as an input. Still speculative, and now less likely: the
>    *base* board drives GPIO42 as FL SV and it works.
>
> The 30-second experiment is unchanged and still nobody has run it: connect the
> arm over USB (`cd arm && pio device monitor -e esp32s3`), press `m` for the live
> switch monitor, and press the X max stop by hand. **Note the arm's `Serial` is
> UART0 on GPIO43/44, not native USB** — plugging into the USB-C port gives a port
> that enumerates, stays silent and ignores you (`gotchas.md` §6).

### TOF1–TOF4 — VL53L0X, arm side

Same 6-pin pinout as the base board (VIN · GND · SCL · SDA · NC · XSHUT).

| Header | Schematic name | XSHUT GPIO | Assigned address |
|---|---|---|---|
| TOF1 | ToF X1 | GPIO18 | `0x30` |
| TOF3 | ToF X2 | GPIO21 | `0x31` |
| TOF2 | ToF Y1 | GPIO6 | `0x32` |
| TOF4 | ToF Y2 | GPIO12 | `0x33` |

Matches `HardwareConfig.h`. **[VERIFIED]** Note the designator order is *not* the
axis order — TOF2 is Y1 and TOF3 is X2.

> ⚠️ **A dead ToF here reads as "clear air."** `TofSensorArray.cpp` substitutes
> 8191 mm on `RangeStatus == 4`, and the air test is `distance > 200`. A
> disconnected or failed sensor therefore tells the arm it may stop extending and
> start flipping. `gotchas.md` §1. Do not debug arm motion without checking these
> first.

### J3 — External Buck 5V and GND

| Pin | 1 | 2 |
|---|---|---|
| Signal | **+5 V in** | GND |

Feeds all four servo headers and both BTS7960 logic supplies. **Unlike the Wheel
Drive PCB's J3, this one is a power input.** See `startup.md` §2.

### Pins the arm board leaves unconnected

`GPIO0 · 19 · 20 · 35 · 36 · 37 · 43 · 44 · 45 · 46`, the DevKit `5V` pad, and
`CHIP_PU`. **[VERIFIED — netlist]**

Note GPIO3 **is** used (`Y_ARM_MIN`) and is an ESP32-S3 strapping pin (JTAG
source select). It is read at reset. A limit switch held closed at power-on pulls
it low — which is the strapping condition. Nothing has been observed to go wrong,
but if the arm ever boots strangely with the Y axis at its minimum stop, look
here first. **[HYPOTHESIS — pin function is verified, the interaction is not]**
