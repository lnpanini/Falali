# ESP32-S3 motor bench — test session handoff

> **Cold-start handoff.** Self-contained: a fresh Claude Code session (or Bryan)
> should be able to pick up the bench work from here with no prior conversation.
> Read §1–§3 before touching hardware.

**Written:** 2026-08-05
**Goal of the session:** get one BLD-120A + motor turning from the ESP32-S3, then
measure **break-away point** and **true top speed** — the two numbers that unblock
everything downstream.

---

## 1. Where things stand

### Working and verified

| | |
|---|---|
| `bench_s3` firmware | flashed, responds to `?`, boots safe (disabled / no brake / 0%) |
| Slew limiter + interlock | verified in the log — SV gated on `enabled && !brake` |
| Serial | `/dev/cu.wchusbserial5B8F0947541` on the **Mac**, 115200, CH343 bridge |
| Logging | `teleop.py --log` writes CSV + raw to `bench_logs/` |
| Host unit tests | 8 suites, 68 tests, green under `-Wall -Wextra -Werror` |

### ✅ RESOLVED 2026-08-05 — motor turns under full EN/BRK/F-R control

**Root cause: the adapter transistors were miswired** — C and E swapped, and the
100 kΩ pulldown on the collector instead of the base. See
`docs/hardware-architecture.md` §4 for the before/after and the diagnostic
signature. Nothing was damaged; the same transistors were reused.

Confirmed working with the EN jumper **removed**, so the safety chain
(`SafetyMonitor`, `safeStop()`, `LinkWatchdog`) has real authority again.



Full chain verified: Mac → ESP32-S3 → transistor adapter → BLD-120A → motor.
The adapter design and the transistor orientation are **good**.

Three things changed between the failing run and the working one. **Which one was
decisive is not yet known** — worth establishing before building adapters 2–4:

1. **BRK was latched on.** `x` E-STOP sets the brake and `teleop.py` fires `x` on
   exit, so a stale brake survives across sessions. The status line now names the
   blocker explicitly (`<-- BLOCKED: brake on, press 'n'`).
2. **P-sv turned to maximum** — the SYS manual's own advice for *"when the motor
   cannot adjust the speed"*.
3. **SV PWM moved 5 kHz → 2 kHz** — 5 kHz was outside the SYS manual's 1–3 kHz range.

Note (1) alone does **not** explain the first failure: `bench_20260805_211945.csv`
shows `EN:on BRK:off duty 4095 ~3.30 V` held 16 s with no movement and the
interlocks all clear. So (2) or (3) — or both — mattered.

**To find out:** with the rig running, set PWM back to 5 kHz and see if it stops;
then restore 2 kHz and back P-sv off. Cheap now, expensive to guess later.

### Working key sequence

```
n  brake off  →  f  forward  →  e  enable  →  3  30% duty
```

---

## 2. Hardware setup

```
ESP32-S3-DevKitC ── adapter (perfboard) ── BLD-120A ── BLDC motor (15:1, 150 mm wheel)
                                              24 V bench PSU
```

### Bench pin map — **NOT the PCB pinout**

| Signal | GPIO | Via |
|---|---|---|
| SV | **4** | 1.5 kΩ series |
| EN | **8** | transistor |
| BRK | **9** | transistor |
| F/R | **15** | transistor |
| COM | GND | plain wire |

> On the fabricated Wheel Drive PCB, **GPIO8/9 are I²C SDA/SCL and GPIO4 is a ToF
> XSHUT line.** The FL wheel there is SV=42, F/R=41, EN=40, BRK=39. See
> `include/pins.h`, which is generated from the netlist. Do not carry the bench
> wiring over.

### Adapter, per control line

```
GPIO ──10 kΩ──┬── Base          Collector → driver input
              │                 Emitter   → COM
           100 kΩ
              │
             COM               SV: 1.5 kΩ series, no transistor
```

**The transistor inverts.** GPIO HIGH = transistor on = driver input pulled to
COM = **asserted**. Firmware handles this via `fal::kControlViaMosfet = true` in
`lib/hal_esp32/Bld120aMotor.h`; `bench_s3_motor.cpp` has the same constants inline.

---

## 3. Facts already established — do NOT re-derive these

Each cost real time to pin down. Sources in `docs/hardware-architecture.md`.

| Fact | Evidence |
|---|---|
| **SV is a high-impedance voltage input** | diode test 2026-08-05 read `OL` both directions. It does *not* load the source. |
| **The driver accepts PWM natively, 1–10 kHz** | manual, Speed Command mode C. No DAC / op-amp / RC filter needed. 20 kHz is out of spec. |
| **EN/BRK/F-R are optocouplers idling ~5 V** | internal pull-up ≈10 kΩ, ≈440 µA when pulled to COM. ESP32-S3 is not 5 V tolerant. |
| **A pulldown cannot replace the transistor** | pin-safe needs R ≤ 19.4 kΩ, opto-off needs R ≥ 32 kΩ. No overlap. |
| **RV pot must be fully ANTICLOCKWISE** | manual states it 3× — external speed control *fails* otherwise. |
| **P-sv is the overload power limit (30–120 W), not a speed trim** | manual §Settings. Wrong value trips the red LED and the motor won't run. |
| **There is no ALM terminal** | control block is only SV/COM/F-R/EN/BRK. "RUN/ALM" is an LED. |
| **The 2.59 V figure is stale** | 2026-07-15, ESP32 **DAC** on the WROOM-32D. Not applicable to the S3. |
| **`pins.h` is now netlist-derived** | the old map was pre-PCB intent and did not match the fabricated board. |

---

## 4. Test procedure

### Step 0 — pre-flight (both are silent failures)

- [ ] **RV pot fully anticlockwise**
- [ ] **P-sv set to the motor's rated watts** — read the nameplate
- [ ] Wheel **off the ground**
- [ ] Hall cable connected: HU/HV/HW **and** REF+/REF− (a BLDC cannot commutate without it)
- [ ] **RUN/ALM LED colour** — green = OK, **red = fault**. The ESP cannot see this.

### Step 1 — verify the adapter (do this first)

Unplug the adapter from the **driver** end; leave it on the ESP. No 5 V present, so
this is safe.

Measure **output pin → COM** while toggling the GPIO (`e`/`d` in the console
toggles EN; `b`/`n` toggles BRK):

| GPIO | Expected | Failure meaning |
|---|---|---|
| LOW | > 1 MΩ | conducting → C/E swapped, or pulldown missing |
| HIGH | < 100 Ω | open → base not connected; few hundred Ω → **C/E swapped** |

Check all three lines. If one is wrong they probably all are.

### Step 2 — verify signals reach the driver

Reconnect, 24 V on, `e` to enable, duty 0. Meter each terminal against COM:

| Terminal | Expected |
|---|---|
| EN (enabled) | ~0–0.2 V |
| BRK (released) | ~5 V |
| F/R (forward) | ~5 V |
| SV at 100 % | ~3.3 V |

### Step 3 — break-away sweep

```bash
cd ~/Documents/GitHub/Falali
/opt/anaconda3/bin/python teleop.py --log
```

`?` → `f` → `e` → then `+` one step at a time from zero.

**Record the first duty % at which the wheel moves at all.** That is break-away.
Fill it into the `measured_rpm` / `note` columns of the CSV as you go — the bench
has no speed feedback until the encoders are wired.

### Step 4 — top speed

`w` for 100 %, time N wheel revolutions with a stopwatch.

```
wheel_mm_s = (wheel_rpm × π × 150) / 60
```

---

## 5. What the numbers decide

**Break-away → is the minimum speed usable?**
Estimated ~0.78 V → ~413 mm/s at the wheel, already above the 300 mm/s target, with
one 20 ms tick moving 8.3 mm against an 8 mm centring tolerance. If confirmed,
that's a **gearing/wheel-diameter problem, not a firmware one**. The driver's PID
speed loop (manual: *"PID speed loop"*) may rescue it — the sweep will show.

**Top speed → `max_lin_mm_s` is currently wrong.**
`config.h` says 300 mm/s; the drivetrain does ~1400 mm/s at full command.
`DeadReckonOdometry` therefore under-estimates every distance by ~4.7×, which will
make `CENTER_X`/`CENTER_Y` overshoot badly.

**Also check:** `rotate_speed` and `centre_speed` are both `0.20` → 0.66 V →
**below the estimated break-away**. If that holds, `ORIENT`/`CENTER_X`/`CENTER_Y`
will never move and will run to their timeouts.

---

## 6. Safety

- **No motor fault detection exists in hardware.** No ALM terminal, and
  `StallDetector` needs encoder feedback that isn't wired yet. **Do not leave the
  rig running unattended.**
- Protection chain, worst case first: driver's P-sv trim (≤8 A, unverified
  position) → 10 A fuse. An 8 A stall sits below the fuse indefinitely while
  ~192 W cooks stationary windings.
- `x` is the E-STOP key. `teleop.py` also fires `x` on exit.
- ESP GND ties to **COM only** — never to DC−.

---

## 7. Useful commands

```bash
pio run -e bench_s3 -t upload            # flash the bench sketch
pio run -e native -t test                # 68 host unit tests
/opt/anaconda3/bin/python teleop.py --log   # console + CSV logging
ls -t bench_logs/*.csv | head -1         # newest session log
```

## 8. Files that matter

| Path | What |
|---|---|
| `docs/hardware-architecture.md` | **authoritative** GPIO map, I²C addresses, BOM, decision log |
| `docs/BLD-120-English-version.pdf` | driver manual |
| `src/bench_s3_motor.cpp` | this bench sketch |
| `include/pins.h` | PCB pin map (netlist-derived) — *not* the bench pins |
| `teleop.py` | console + logger |
| `bench_logs/` | session data |
