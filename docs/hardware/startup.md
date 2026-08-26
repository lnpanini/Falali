# Powering up Falali

**Read this before you connect the battery.** It is the only document that
describes the power system, because none of it is visible in the KiCad files —
no board in this project has a power input connector.

Tags follow `docs/gotchas.md`: **[VERIFIED]** measured or read from source,
**[OBSERVED]** it happened and the evidence is quoted, **[HYPOTHESIS]** a model
that fits and has not been tested. Where a hypothesis matters, the experiment
that would kill it is written next to it.

---

## 1. The safety facts, before anything else

### The big red button is not an emergency stop **[VERIFIED — Bryan, 2026-08-16]**

It looks like an E-stop. It is wired as an **enable**.

> **Pressing it turns the 12 V arm rail ON. Releasing it turns that rail OFF.**

Every reflex you have about mushroom buttons is backwards here. In an emergency,
the instinct is to slap the button down — on this machine that **energises** the
arm rather than killing it. To actually stop the arm you must **twist and pull
the button out.**

As wired, it is a power switch with a misleading head.

**What changing it would involve:** the fitted part is a **1NO 1NC DPST**
latching mushroom switch, so both contact types are physically present and the
**NC contact is currently unused**. The rail sits on NO. Moved to NC the
behaviour inverts to what most people expect — released (out) = rail live,
pressed (in) = rail dead — and nothing else in the system changes.

**What the button does not cover, either way:** the **24 V motor rail is not
switched by it at all.** A conventional E-stop would break 24 V too. The only
thing that stops the wheels is the firmware, and — see §5 — the firmware is not
running for the first part of the power-up.

### The wheels are free until the firmware boots **[VERIFIED — netlist + polarity table]**

This falls out of the Driver PCB's design and is worth internalising.

Each control line reaches the BLD-120A through a 2N3904 with a **100 kΩ
base-to-COM pulldown**. While the ESP is unpowered, in reset, or simply not yet
running your code, its GPIOs are high-impedance, the pulldown holds the
transistor **off**, and the collector is released. Released means the driver's
own internal pull-up wins. Cross-referencing the asserted-state table in
`hardware-architecture.md` §3:

| Line | Transistor off → driver pin | Meaning |
|---|---|---|
| EN | released (high) | **motor disabled** |
| BRK | released (high) | **brake released** |
| F/R | released (high) | forward |

So the safe half is real — an unbooted ESP cannot drive the motors. But the
brake is **also** released, so **the robot is a free-rolling 4-wheel trolley from
the moment 24 V comes up until firmware takes over.**

On a bench: wheels off the ground, or chocked. On the floor: on a level surface,
against nothing you mind it touching. Do not power up on a ramp.

**The arm behaves the opposite way.** Its extension motors are **self-locking
worm gearboxes**, so the arm holds position with no power at all and does not
back-drive under load. Cutting the 12 V rail freezes the arm where it is; it does
not drop. The flipper servos are not self-locking — `stopAll()` cuts their PWM
and releases them, so they settle wherever gravity takes them (`gotchas.md` §3).

### There is no motor fault detection in hardware **[VERIFIED]**

The BLD-120A exposes no ALM terminal — "RUN/ALM" is an LED, not an output, and
the Wheel Drive PCB has no ALARM net. Protection is, in order:

1. the driver's **P-sv** overload trim (set it to the motor's rated watts),
2. the pack fuse,
3. ~~software current-sense (`cfg::kWheelStallAmps`)~~ — **this layer does not
   currently exist.** The ACS758 sensors are installed inline on the motor-supply
   negative but their **outputs are not connected to the board**
   (`components.md` §4). `StallDetector` is reading an ADC that sees nothing.

So in practice the protection is **P-sv and the fuse. That is all.**

**Do not run the drivetrain unattended**, and treat a stalled wheel as something
only a human will notice.

---

## 2. The power tree

```
36 V 10 Ah battery pack  (custom-built, internal BMS — this is what trips on inrush)
        │
        └── main switch  (40 A)
              │
              ├── Buck 1 ──► 24 V, 30 A ──► 4× BLD-120A driver  (wheel motors)
              │                        └─ ACS758 sensors sit inline on the NEGATIVE
              │                           return — signal NOT wired (components.md §4)
              │
              ├── Buck 3 ──►  5 V, 20 A ──► Arm PCB J3 terminal block
              │                        └► 4× GX3345BLS servo + BTS7960 logic
              │
              └── red mushroom button (latching, 1NO 1NC — gates the RED/positive
                    │                    conductor on the buck's 36 V INPUT side)
                    │
                    └── Buck 2 ──► 12 V, 25 A ──► arm wormgear motors, self-locking
                                                   (through the BTS7960 H-bridges)

   ESP32-S3 ×2 (base + arm)  ──  USB POWER BANK.  Nothing to do with the pack.
```

**[VERIFIED — Bryan]** all three rails and their ratings, their loads, the 40 A
main switch, the ESPs running from a USB power bank, and — settling a question
that was open until 2026-08-17 — that the button gates the **input** side of the
12 V buck.

That last point matters: gating the input is what makes the inrush staggering in
§3 real. Had it gated the output, the buck would still charge its input
capacitors at main-switch time, which is the event that trips the BMS, and the
staggering would have been decorative.

### Two terminal blocks, two completely different jobs

Both boards carry an identical Phoenix MKDS 5.08 mm 2-position block. They are
**not** interchangeable, and mixing them up puts 5 V onto a ground net.

| Board | Ref | Schematic name | What it actually is |
|---|---|---|---|
| Wheel Drive PCB | **J3** | "Common Ground" | **Ground bond only.** Pin 1 is the board ground net; **pin 2 is unconnected** — it is a spare post, not a supply input. **[VERIFIED — netlist]** |
| Arm Subsystem PCB | **J3** | "External Buck 5V and GND" | **5 V servo supply in.** Pin 1 = +5 V, pin 2 = GND. Feeds all four servo headers and both BTS7960 logic pins. **[VERIFIED — netlist]** |

### Wheel Drive J3 is not optional **[VERIFIED — reproduced 2026-08-06]**

The base ESP is USB-powered and the drivers run off the 24 V buck. Those are two
different grounds. **J3 is the only thing that bonds them.**

Without it the ESP boots `rst:0x15,boot:0x3 (DOWNLOAD(USB/UART0))` every time —
GPIO0 has no sane reference and reads low at reset, so the application never
runs. It presents as a strapping-pin fault and sends you hunting for solder
bridges on a pin the netlist leaves deliberately unconnected.

The isolating test takes one move: lift the DevKit off the PCB and reset it
standalone.

| Code | Meaning |
|---|---|
| `boot:0x8 (SPI_FAST_FLASH_BOOT)` | normal — your app is running |
| `boot:0x3 (DOWNLOAD(USB/UART0))` | GPIO0 was low at reset; app never runs |

`0x8` off the board and `0x3` on it localises the fault to the board — which
means the ground bond.

---

## 3. Why the order matters: inrush

**[HYPOTHESIS — Bryan's working model, fits the behaviour, not instrumented]**

All three buck converters sit across the 36 V pack. Each one's input capacitance
charges from empty when the main switch closes, and three of them together draw a
current spike large enough that **the pack's BMS reads it as a short and latches
off.** The symptom is a robot that appears completely dead the instant you switch
it on.

The mitigation already built into the machine is to **stagger** the inrush: the
12 V buck was moved behind the red button, so only two bucks energise at
main-switch time, and the third joins a moment later when you press it.

*Experiment that would settle it:* clamp a current probe on the pack lead and
switch on. A single spike into the tens of amps for a few milliseconds confirms
inrush. If the trip instead correlates with pack voltage sagging, the BMS is
cutting on undervoltage and the cause is a tired pack — which has a different
remedy (pre-charge resistor vs. new cells).

**If it trips anyway:** switch off, wait for the BMS to reset, and bring the
main switch up again. Do not hold the switch part-closed to "ease it in" — arcing
across a partly-closed contact at 36 V will pit it.

---

## 4. Start-up sequence

Do these in order. The order is the point.

### 0 — Pre-flight

- [ ] **Wheels off the ground, or the robot chocked.** §1: the brake is released
      until firmware boots.
- [ ] Wheel Drive **J3 bonded to the motor supply ground.** §2.
- [ ] Red button **out** (12 V arm rail dead) — so the arm cannot move while you
      are near it.
- [ ] Nothing under the arm's travel.
- [ ] All four Driver PCBs seated, right way round. See `boards.md` — J1 is the
      ESP side, J2 the driver side, and they are **not** symmetric.

### 1 — Main switch on

24 V and 5 V come up. 12 V does not.

**Expected:** each BLD-120A shows a **blinking green LED**.

### 2 — Check the four driver LEDs

**[OBSERVED — Bryan]** A driver that comes up without its green LED blinking
needs an **off/on cycle of its own power** before it will run. One driver
misbehaving is normal-ish; all four is a supply problem, not four coincidences.

| What you see | What it means | Do |
|---|---|---|
| Green blinking | healthy, ready | continue |
| Green absent / not blinking | driver did not initialise | power-cycle **that driver**, then re-check |
| **Red** LED | overload trip — usually **P-sv** set below the motor's rated watts | fix P-sv before going further (§6) |

Do not proceed to §5 with a driver showing red. It will not run, and you will
spend the afternoon debugging firmware that is working correctly.

### 3 — Press the red button

12 V comes up; the arm extension motors now have power. Only do this once you
are clear of the arm.

### 4 — Power the ESPs over USB

Base and arm each take their own USB cable. Both boards leave the DevKit's `5V`
pin **unconnected** **[VERIFIED — netlist, both boards]**, so USB is genuinely
the only supply path; there is no alternative to fall back on.

Watch the base console for the boot banner. `boot:0x8` is what you want (§2).

### 5 — Pair the gamepad and verify

The base runs Bluepad32 and expects an **Xbox BLE** pad. Once connected, check
before trusting it:

- a small drive command moves the robot in the direction you expect;
- all four wheels respond;
- the console reports the base's own MAC matching `SELF_ESP_MAC`. A mismatch
  means arm commands will go nowhere with nothing on screen looking wrong
  (`gotchas.md` §5).

---

## 5. Shut-down

1. Release the red button (twist-and-pull) — arm rail dead.
2. Main switch off.
3. Unplug the ESPs' USB.

Between steps 1 and 2 the wheels lose firmware braking and **free-roll again**.
Chock before you walk away.

---

## 6. Driver trims — there are **three**, set them once per driver

Earlier documentation listed two. The BLD-120A has **three** adjusters, and the
third was missed entirely.

| Trim | Where | Function | Required setting |
|---|---|---|---|
| **RV** | top of the board, blue pot | onboard speed pot; **sums with** the external SV input | **fully anticlockwise (left).** External speed control *fails* otherwise — stated three separate times in the manual |
| **P-sv** | rotary dial, front face | overload trip point | **5 A** — see below |
| **ACC/DEC** | rotary dial, front face | acceleration & deceleration ramp, **0.3 s – 15 s** | **no value specified by the manual** — see below |

### P-sv → 5 A (equivalently 120 W at 24 V)

The two datasheets label this dial differently, and it caused the confusion in
the older notes:

- `BLD-120-English-version.pdf` shows it marked **"Peak Power, Unit: W"**, 30–120 W.
- `BLDC-BLD120A-...-specs.pdf` describes it as **"P-sv Current … set range 1.6 A–8 A"**.

**These are the same setting.** The motor is **120 W at 24 V = 5 A**, so set the
dial to 5 A, or to 120 W if yours is the watt-marked revision. Note 120 W is the
**top** of the watt scale. **[VERIFIED — both datasheets + motor rating]**

Getting it wrong is a specific, recognisable failure: *"Why does the motor not
run since the red light is on? Check whether the power setting is too low or too
high."* — **too high is also a fault**, not just too low.

### ⚠️ ACC/DEC — an unrecorded variable

> **[VERIFIED — datasheet]** *"This potentiometer can be used for adjusting
> acceleration and deceleration time directly … The range can be set is:
> 0.3s–15s."*
>
> **The positions of all four dials are unrecorded.** At anything but the short
> end of that range, the driver ramps every speed change over seconds, and the
> base firmware assumes otherwise:
>
> - `DeadReckonOdometry` integrates the **commanded** velocity and assumes the
>   wheel follows immediately (`gotchas.md` §12). Under a multi-second ramp the
>   robot travels less than the estimate says during every start and stop.
> - The docking state machine issues short corrective moves. A move shorter than
>   the ramp never reaches its commanded speed.
> - The observable result is indistinguishable from stiction — a wheel that
>   "does not respond" to small commands.
>
> The trim's existence and range are verified; its current setting is simply
> unknown, so whether any of the above is actually happening is
> **[HYPOTHESIS]**. *Reading the four dials establishes whether it is a factor at
> all. Changing them and re-running a dock would show whether it accounts for the
> low-speed behaviour in `gotchas.md` §12.*

---

## 7. When it does not start

| Symptom | Most likely cause | Check |
|---|---|---|
| Totally dead at main switch | BMS latched on inrush (§3) | switch off, wait, retry |
| ESP boots to `boot:0x3` forever | ground bond missing | Wheel Drive J3 (§2) |
| Driver green LED not blinking | driver needs a power cycle | cycle that driver (§4.2) |
| Driver red LED | P-sv below motor rating | §6 |
| Motor enabled but does not turn | RV pot not fully anticlockwise | §6 |
| One motor dead, others fine | Driver PCB adapter fault | `boards.md` — the 0.7 V base test |
| Board silent, esptool says `No serial data received` | HWCDC blocked on a full TX buffer | replug USB; add `Serial.setTxTimeoutMs(0)` |
| Board drops into bootloader when a script opens the port | DTR is the IO0 strap on USB-Serial-JTAG | open the port with **DTR and RTS low** |
| Wheels below a certain speed do nothing at all | below break-away; they do not run slowly, they do not run | `gotchas.md` §12 |

---

## Gaps in this document

Written down rather than guessed, so nobody mistakes an absence for a fact:

- [ ] Battery pack internals. It was **custom-built**, so cell count, chemistry,
      BMS ratings and charge profile are all unknown. This is the single largest
      undocumented risk in the machine.
- [ ] Fuse rating and location. (`hardware-architecture.md` §6 mentions a 10 A
      fuse in the trip chain; where it physically sits is unrecorded.)
- [ ] Which power bank the ESPs run from, and its capacity.
- [ ] **Where the four ACC/DEC trims are currently set** (§6) — unrecorded, and
      the setting bears on the low-speed behaviour in `gotchas.md` §12.
- [ ] The 5 V buck's true minimum input voltage (`components.md` §5).
