# Term paper — electronics & power section, rewritten

**Scope:** §II.C.3 (Electronics & Motion Control System), the battery-sizing part of §III.A,
a new electrical results subsection for §IV, and a limitations paragraph for §V.
Everything here is drop-in prose for the paper, followed by a change log and the
list of numbers still to be measured.

**Sourcing:** every figure is traced to `docs/hardware/components.md`,
`docs/hardware/startup.md`, `docs/hardware-architecture.md` or `docs/gotchas.md`.
Values that the repo marks as unrecorded are written as **[EST]** in the tables and
must be either measured or removed before submission — see *Open numbers* at the end.
Do not submit an **[EST]** as a fact.

---

## §II.C.3 — Electronics & Motion Control System (condensed, ~300 words)

*(Use this version if the section must stay to one paragraph block. The long-form
version follows below — keep only one.)*

The electronics architecture is a two-controller distributed system built on the staged
power tree of Fig. 6, which separates the 36 V main power system from a USB-fed logic
system joined to it at a single point. Three custom PCBs were designed and fabricated
for it: a base Wheel Drive board (Appendix A), an Arm Subsystem board (Appendix B), and
four identical Driver adapter boards (Appendix C). A custom 36 V, 10 Ah lithium-ion pack
with integral BMS feeds a 40 A main switch and a reverse-polarity and bulk-capacitance
stage supplying the main 36 V bus. Three independently protected branches descend from
that bus: a 30 A fuse ahead of the 36→24 V converter feeding the four BLD-120A brushless
drivers; a 20 A latching enable switch ahead of the 36→12 V converter feeding the two
BTS7960 H-bridges and arm-extension motors M5–M6; and an 8 A fuse ahead of the 36→5 V
converter feeding the PCA9685 expander and four GX3345BLS flipper servos S1–S4 on the
arm board. Each driver carries a further 10 A fuse on the 24 V sub-bus, so a single
motor fault clears locally, and each driver's P-sv overload trim is set to the motor's
rated 5 A (120 W at 24 V). Gating the 12 V branch at its converter input also staggers
inrush: all three converters charging together latched the pack BMS, whereas energising
the third separately keeps the transient below it. The drivers' optocoupler inputs idle
near 5 V, so each control line is switched by a 2N3904 on its Driver board, whose 100 kΩ
pull-down holds every motor disabled while a controller is unpowered or in reset. Each
branch returns through its own ground to the main VBat− star bus, meeting the logic
ground plane at the single tie of Fig. 6; without that bond, made at the Wheel Drive
board's J3 terminal, the base controller does not boot.

---

## §II.C.3 — Electronics & Motion Control System (long form)

*(Replaces the existing subsection in full.)*

The electronics architecture is a two-controller distributed system built around a
staged step-down power tree (Fig. 6). Control authority is split between a **base
controller**, which owns the drivetrain and the docking state machine, and an **arm
controller**, which owns the extension and clamping hardware. Both are ESP32-S3-DevKitC-1
(N16R8) modules.

**Power distribution.** DC power is supplied by a custom-built 36 V, 10 Ah lithium-ion
pack with an integral battery management system (BMS). The pack feeds a 40 A main switch,
from which three buck converters step 36 V down to the three service rails in parallel:
a **24 V** rail supplying the four BLD-120A brushless motor drivers, a **12 V** rail
supplying the two arm-extension wormgear motors through their BTS7960 H-bridges, and a
**5 V** rail supplying the four GX3345BLS (45 kg·cm) flipper servos together with the
logic supplies of both H-bridges. Rail assignments and loads are given in Table 5.
The parallel topology was chosen over a cascaded 36 → 24 → 12 → 5 V chain so that a
fault or transient on one rail cannot propagate to the others, and so that each
converter's current rating is set only by its own load.

The two ESP32-S3 controllers are powered separately over USB rather than from the pack.
This isolates the logic domain from motor-supply transients — the dominant noise source
in the system — at the cost of an external supply requirement that is addressed in §V.
Because the controllers and their sensors are not pack-fed, the logic domain contributes
no load to the battery budget in §III.A.

**Protection and sequencing.** Protection is layered. Each BLD-120A carries a hardware
overload trim (P-sv) set to the motor's rated 5 A ≡ 120 W at 24 V, which trips the driver
before its 8 A continuous rating is exceeded. A pack fuse sized per Table 6 protects the
main conductor, and the 40 A main switch doubles as the manual isolator. Signal-level
protection is provided by the driver interface described below, which holds the motors
disabled whenever the controller is unpowered or in reset.

Power-up sequencing is a deliberate design feature rather than an operating convenience.
All three converters present their input capacitance to the pack simultaneously at
switch-on, and the resulting inrush transient was observed to trip the pack's BMS into a
latched fault. The 12 V branch was therefore moved behind a latching panel switch so
that only two converters energise at main-switch time and the third is brought up a
moment later, staggering the transient below the BMS threshold. A pre-charge resistor
is identified in §V as the more robust permanent fix.

**Motor drive interface.** Each BLD-120A is commanded over five lines — speed (SV),
direction (F/R), enable (EN) and brake (BRK) referenced to the driver common (COM). The
driver's control inputs are optocoupler inputs idling near 5 V, and the ESP32-S3 is not
5 V tolerant, so the two domains are separated by a custom four-off adapter PCB. Each
control line drives a 2N3904 in open-collector configuration through a 10 kΩ base
resistor, with a **100 kΩ base-to-common pull-down** that holds the transistor off
whenever the GPIO is high-impedance. This makes the interface fail-safe by construction:
with the controller unpowered, unflashed or in reset, all four motors read as disabled.
SV is driven directly through a 1.5 kΩ series resistor as a 2 kHz PWM waveform, within
the driver's specified 1–3 kHz input range and 2–90 % duty band; no DAC, filter or
external speed pot is required.

**Sensing.** The base carries four upward-facing VL53L0X time-of-flight sensors, one at
each corner of the chassis, which are polled in sequence to infer the robot's position
and orientation relative to the underside of the trolley during approach. All four share
a single I²C bus and the same factory address, so each sensor's XSHUT line is held in
reset and released one at a time at boot, allowing a unique address to be assigned per
corner. A BNO085 inertial measurement unit on the same bus supplies orientation, which
the firmware fuses with commanded wheel velocity to maintain a dead-reckoned position
estimate between sensor updates. Four further ToF sensors, eight limit switches and a
PCA9685 PWM expander sit on the arm controller, where the ToF sensors detect the trolley
edge and the limit switches confirm contact at the end of retraction.

Per-wheel current sensing is provided for by four ACS758LCB-050B Hall-effect sensors
inline with the motor-supply return, feeding a 16-bit ADS1115 ADC. The ±50 A part was
selected over a more sensitive ±5 A alternative because the driver is rated to 30 A
instantaneous: a 5 A sensor saturates during precisely the stall and inrush events worth
detecting, whereas Hall noise scales with sensitivity, so the resolution penalty of the
larger part is only about 1.5×. The signal path is not yet commissioned (§V).

**Command and communications.** Operator commands originate from an Xbox controller
paired to the base over Bluetooth Low Energy. The base then issues clamping commands to
the arm controller over **ESP-NOW** on WiFi channel 1, using a deliberately minimal
four-command protocol — `grab`, `release`, `home_setup`, `estop` — with acknowledgement
replies (`done_*`, `busy`, `stopped`, `failed:<reason>`). A radio link was adopted after
a three-wire UART between the controllers proved unreliable in the assembled machine. The
trade-off is explicit: the BLE gamepad stack and the WiFi radio now share the same 2.4 GHz
front end through the coexistence scheduler, which costs a small amount of gamepad
responsiveness. Two interlocks bound the failure modes — a `busy` reply suppresses the
next ordinary command so that a queued instruction cannot execute against a state that
has since changed, and the E-stop command is exempt from that suppression on the
principle that a stop a status message can swallow is not a stop.

---

## §III.A — Electrical modelling (replaces the "Battery Sizing" paragraph)

*(Insert after the motor/gearbox paragraph, before Thermal Analysis.)*

**Rail loading.** The load on each rail is fixed by the components it feeds (Table 5).
The 24 V rail dominates: four 120 W motors at their rated 5 A draw 20 A, or 480 W, with
all four wheels working. The four drivers can supply 8 A each, giving 32 A of installed
capacity against a 20 A demand, so the drivers are not the binding constraint — the
converter is.

**TABLE 5: Rail allocation and loading**

| Rail | Loads | Continuous load | Worst case | Basis |
|---|---|---|---|---|
| 24 V | 4× BLD-120A + BLDC gearmotor | 20 A / 480 W | 32 A driver limit; 30 A instantaneous < 3 s | Motor rated 5 A at 24 V |
| 12 V | 2× wormgear extension motor via BTS7960 | **[EST]** 10 A / 120 W | H-bridge rated 43 A | **[EST]** — motor spec unrecorded |
| 5 V | 4× GX3345BLS servo + 2× BTS7960 logic | **[EST]** 4 A / 20 W | **[EST]** 10 A / 50 W (four servos stalled) | **[EST]** — servo datasheet unconfirmed |
| USB | 2× ESP32-S3 + all sensors | ~1 A total | — | Not pack-fed; excluded from battery budget |

**Pack current by operating phase.** Peak demand is phase-dependent, because the docking
sequence holds the robot stationary while the arms actuate; drive and clamp loads are not
commanded concurrently. Converting rail power to pack current must include converter
efficiency, taken as η = 0.85:

    I_pack = P_rail / (V_pack × η)

| Phase | Rail power | Pack current |
|---|---|---|
| Driving under load (four wheels at rated torque) | 480 W | 480 / (36 × 0.85) = **15.7 A** |
| Docking (arms extending, servos flipping) | 140 W **[EST]** | **4.6 A** |
| Absolute worst case (all loads concurrent, not commanded) | 650 W **[EST]** | **21.2 A** |

The 15.7 A driving figure, not the 21.2 A arithmetic worst case, is the number the pack
and its conductors must sustain continuously.

**Battery sizing.** Taking a duty-weighted average draw of 250 W over a working cycle —
the drivetrain loaded for roughly half the cycle, plus docking activity — and requiring
one hour of runtime with a 20 % state-of-charge reserve (usable depth of discharge
DoD = 0.8):

    C = P × t / (V × DoD × η) = 250 × 1 / (36 × 0.8 × 0.85) = 10.2 Ah

The selected 36 V 10 Ah pack (360 Wh) therefore sits essentially at the sizing point,
delivering approximately 59 minutes at this duty. Sizing to the arithmetic worst case
instead would demand a far larger pack for a condition the control sequence never
commands; the one-hour figure is quoted against realistic duty and is stated as such.
Note that including converter efficiency raises the requirement by about 18 % over the
ideal-conversion calculation, which is the difference between a pack that meets the
requirement and one that narrowly misses it.

**Fuse and conductor sizing.** Fuses are sized at approximately 1.5× the continuous
current of the branch they protect and below the ampacity of the conductor they sit on:

**TABLE 6: Overcurrent protection**

*(Values as drawn in Fig. 6. Branch currents are referred to the 36 V side, since all
three fuses sit upstream of their converters.)*

| Element | Continuous current | Device (Fig. 6) | Note |
|---|---|---|---|
| Drivetrain branch | 15.7 A | 30 A fuse | ~1.9× margin; carries converter inrush as well as load |
| Each BLD-120A | 5 A rated at 24 V | 10 A fuse ×4 | **Must be time-delay** — the driver draws 30 A instantaneous for < 3 s, which opens a fast-blow 10 A on every start |
| Arm branch | 3.9 A **[EST]** | 20 A enable switch | **No fuse fitted.** A switch rating is not overcurrent protection — a ~10 A fuse belongs here |
| Servo branch | 1.6 A **[EST]** at 36 V | 8 A fuse | Sized ~5× above the branch load; ~3 A would actually protect it |
| Main conductor | 15.7 A | 40 A main switch | Switch is the isolator; branch fuses provide selectivity below it |
| Pack conductor gauge | 15.7 A | **[EST]** ≥ 10 AWG | Ampacity plus derating in an enclosed chassis |

Protection is selective by design: the per-driver fuses are the smallest devices in the
chain and clear first, so a single seized wheel isolates that motor while the remaining
three continue, and the 30 A branch fuse acts only on a fault involving the converter or
the sub-bus itself.

The layered scheme means an overload on a single motor is caught by that driver's own
P-sv trim in isolation, while the pack fuse protects only against a fault severe enough
to involve the main conductor.

---

## §IV — new subsection C: Electrical characterisation

*(Insert after the existing experiments table. Table 7 reports what has been established
on hardware; Table 8 states the instrumentation still outstanding, which is the honest
position and is better than an empty results section.)*

**TABLE 7: Electrical results obtained**

| Test | Objective | Result |
|---|---|---|
| Drivetrain speed ceiling | Establish the geometric maximum from gearing | 3000 RPM ÷ 15 over a 150 mm wheel = 1571 mm/s; the configured 1231 mm/s command limit is 78 % of ceiling, confirming it is physically attainable |
| Break-away speed | Find the minimum commanded speed at which wheels turn | ≈ 413 mm/s; wheels do not rotate slowly below this, they do not rotate at all — attributed to static friction and load, not to the drive electronics |
| Power-up transient | Determine whether the BMS trip at switch-on is inrush-related | Trip reproduced with all three converters energising together; eliminated by staggering the 12 V branch behind a latching switch |
| Logic/power ground bond | Diagnose repeated controller boot failures | With the USB-powered controller and the 24 V-powered drivers on separate grounds, the controller booted to download mode on every reset; bonding the two grounds at the board terminal restored normal boot |
| Driver interface verification | Confirm the open-collector adapter switches correctly | Base-to-common clamps at ≈ 0.7 V when asserted; a non-clamping junction reading 1.68 V identified a transposed collector/emitter on the first assembly |
| Docking sequence, end-to-end | Confirm sensing and inter-controller command path under real conditions | Full approach, edge detection and clamp cycle completed on hardware; radio link stable after command pacing was added |

**TABLE 8: Instrumentation planned**

| Test | Method | Validates |
|---|---|---|
| Pack current under load | Clamp meter on the pack lead, driving loaded and unloaded | The 15.7 A drive-phase prediction and the 25 A fuse rating |
| Runtime to reserve | Log pack voltage at typical duty until 20 % SoC | The 10.2 Ah sizing calculation |
| Rail sag at peak | Meter 5 V at the arm terminal block during a four-servo flip; 24 V during four-wheel break-away | Converter current ratings, which are currently unrecorded |
| Thermal soak | IR thermometry of drivers and converters after one hour of continuous operation | The passive-heatsink claim in §III.A, presently unmeasured |
| Radio link quality | Command-to-motion latency and dropout rate over a docking cycle, with BLE and WiFi coexisting | The cost of the ESP-NOW link accepted in §II.C.3 |
| Current-sense commissioning | Connect the four ACS758 outputs, then calibrate the stall threshold against a loaded trolley | Restores the only drivetrain fault signal in the design |

---

## §V — limitations paragraph to add

*(Insert into the Discussion, before the closing paragraph. Stating known faults with
their fixes reads as engineering maturity; omitting them and having a grader find them
does not.)*

Three limitations in the present build are identified and carried forward. First, the
panel-mounted latching switch that gates the 12 V arm rail is wired to its
normally-open contact, so pressing it energises the rail rather than isolating it, and
it does not switch the 24 V traction rail at all. It is therefore an enable, not an
emergency stop; moving the rail to the normally-closed contact and extending the
interlock across the traction rail is a single-wire change and is the first item of
future work. Second, the four current sensors are installed in the motor return path but
their signal outputs are not yet connected, so the software stall detector currently
observes no signal and drivetrain protection rests on the driver overload trims and the
pack fuse alone. Third, the two controllers are powered externally over USB rather than
from the pack; a dedicated isolated logic rail is required before the platform can
operate untethered for a full shift. None of these affects the results reported in §IV,
but each bounds the conditions under which the prototype should be operated.

---

## Change log — what this rewrite fixes

| # | Original text | Change | Why |
|---|---|---|---|
| 1 | "The sensors when excluded in order inform the controller … relative to the trolley/" | Rewritten as a complete sentence with the XSHUT addressing explained | The sentence was ungrammatical and ended mid-clause |
| 2 | Peak current stated as 20 A, then calculated as 22.8 A | Single phase-based table; 15.7 A continuous, 21.2 A arithmetic worst case | The paper contradicted itself two lines apart |
| 3 | Servos budgeted at 4 × 25 W | Rewritten around the GX3345BLS 45 kg·cm servos actually fitted, with nominal and stall figures separated | The old figure matched no fitted part |
| 4 | 820 / 36 = 22.8 A | η = 0.85 included in every rail-to-pack conversion | Ignoring converter loss understates pack current by ~18 % |
| 5 | Battery pack described as "2–3 C (20–30 A) discharge" | Described as custom-built with an integral BMS; no discharge rating claimed | The repo records the pack as custom with **no documented C-rating**. The figure could not be supported |
| 6 | Inter-controller link absent; "shared I2C bus" implied it | ESP-NOW link, four-command protocol, interlocks, and the coexistence trade-off stated | The link is the central architectural decision of this section and was missing. I²C is the *sensor* bus |
| 7 | No power budget table | Tables 5 and 6 added | Every other analysis section carries a variables table; this one did not |
| 8 | "Fuses were sized according to the maximum safe as well as operating loads" | Explicit 1.5× rule, values, and conductor ampacity | The original asserted a method without stating it |
| 9 | "Reverse current protection was designed into the power system" | Removed | No such component appears anywhere in the hardware documentation. Re-add only if one is physically fitted, with its part number |
| 10 | Circuit breaker described as main switch | 40 A main switch, as built | Terminology matched neither the part nor the drawing |
| 11 | Motor drive interface not described | Adapter PCB, fail-safe pull-down, 2 kHz PWM within the 1–3 kHz spec | This is the section's strongest original engineering content and was entirely absent |
| 12 | ESP32 power source unstated | Stated as USB, with the isolation rationale and the limitation | A grader will ask where the logic power comes from |
| 13 | Inrush/BMS behaviour unstated | Presented as a designed sequencing decision | Turns an operational quirk into a defensible design choice |
| 14 | Wheel diameter 152 mm | 150 mm | Repo records 150 mm; the speed ceiling arithmetic depends on it |
| 15 | "powers distribution", "convertors", "X-box", "low power Bluetooth", "4 … at each corner" | Corrected throughout | — |
| 16 | No electrical results | §IV.C added, split into obtained and planned | The gap you identified |

---

## Open numbers — measure or delete before submission

Each of these appears as **[EST]** above. An estimate presented as a measurement is a
much worse error than an acknowledged gap.

1. **Buck converter models and current ratings** (all three). Printed on the units —
   read them off. The 24 V unit must carry 20 A and the 5 V unit must survive four
   servos flipping together; if either is undersized, that is itself a finding worth
   reporting.
2. **Arm wormgear motor rating** (voltage, current, power). Sets the 12 V row of Table 5.
3. **GX3345BLS stall current at 5 V.** Determines the 5 V worst case and whether the
   rail is adequately sized.
4. **Pack fuse rating and physical location.** The repo mentions a 10 A fuse; a 10 A
   fuse on the pack conductor would open during normal four-wheel driving at 15.7 A.
   Either it protects a branch rather than the main, or it is wrong. Resolve this
   before submitting Table 6 — it is a genuine safety item, not just a paper issue.
5. **Battery pack internals** — cell count, chemistry, BMS continuous and peak rating.
   Until these exist, no C-rating claim can appear in the paper.
6. **Pack conductor gauge.** Measure and confirm against 15.7 A continuous.

## Appendix contents and labels

The paragraph cites three appendices. Label them this way in the paper so the callouts
resolve, and put schematic before layout within each:

| Label | Board | Files in `KiCad/` | What the text points at |
|---|---|---|---|
| **Appendix A** | Wheel Drive PCB (base) | `Wheel Drive PCB/` — schematic PDF + layout | J3 ground-bond terminal; J4–J7 driver headers; I²C bus and ToF XSHUT lines; ADS1115 and current-sense headers |
| **Appendix B** | Arm Subsystem PCB | `Arm Schematic.pdf`, `Arm PCB Layout Draft Size.pdf` | J3 5 V servo supply terminal; BTS7960 headers J1/J2; SW1–SW8 limit switches; ToF headers |
| **Appendix C** | Driver adapter PCB (×4) | `Driver PCB/` — schematic + layout | The 2N3904 open-collector stage, 10 kΩ base, 100 kΩ pull-down, 1.5 kΩ SV series resistor |

Two notes on presenting them:

1. **Appendix C is the one that earns marks.** It is the only board whose design contains
   an argued engineering decision — the fail-safe pull-down — so give it a caption that
   states that, rather than letting it sit as an unlabelled schematic.
2. **The two custom KiCad libraries are missing** from the repo (`My_30.007_Library`,
   `HengLi's Footprint Library`), which affects re-fabrication but not the exported PDFs
   already in `KiCad/`. Export appendix figures from those PDFs, not from a fresh
   schematic→layout sync.

## Figure 6

The existing power-architecture diagram needs three corrections to match this text:
the 12 V branch behind the latching switch (drawn on the converter **input** side),
the two controllers shown on USB and outside the pack tree, and the ESP-NOW link drawn
between the controllers in place of any wired connection.
