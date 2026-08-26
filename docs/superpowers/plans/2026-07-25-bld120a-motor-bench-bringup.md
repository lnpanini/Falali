# BLD-120A Motor Bench Bring-up — Handoff Plan

> **Cold-start handoff.** This was written because the chat history is being
> reset. It is self-contained: a fresh agent (or Bryan) should be able to pick
> up the bench work from here with no prior conversation. Read it top to bottom
> before touching hardware.

**Goal:** Validate the **BLD-120A** BLDC driver's control interface (SV / F-R /
EN / BRK) on the bench, driven by a **plain ESP32-WROOM-32D devkit**, before
trusting the 4-motor wiring on the production S3 board (`include/pins.h`).

**Status as of 2026-07-25:** firmware written, built, flashed, and the console
+ safety interlock are verified on hardware. **The motor has NOT been spun by
the agent** — enabling the driver is a hands-on-the-PSU step left to Bryan.

---

## What exists (files)

| File | State | Purpose |
|---|---|---|
| `src/bench_motor.cpp` | **new, untracked** | The bench firmware. Single motor, serial-console control. |
| `platformio.ini` → `[env:bench_motor]` | **new, modified** | Build env. `board = esp32dev`, CH340 UART, `SerialCommands` dep. |
| `bench_motor_test/bench_motor_test.ino` | **untracked, SUPERSEDED** | Old Arduino-IDE sketch this replaced. Safe to delete once `src/bench_motor.cpp` is trusted. Not deleted yet in case Bryan still flashes it from the IDE. |

Build/flash/monitor:
```bash
pio run -e bench_motor -t upload && pio device monitor -e bench_motor
```

Both `platformio.ini` and `src/bench_align.h` also show as modified in git from
**earlier, unrelated** work — do not assume those `bench_align.h` changes are
part of this task.

---

## Hardware

- **MCU:** ESP32-WROOM-32D devkit. Chip verified on the wire via esptool:
  **ESP32-D0WD rev v1.0**, MAC `4c:11:ae:b3:9c:18`, on `/dev/cu.wchusbserial310`
  (CH340 UART bridge — NOT native USB, so **no** `ARDUINO_USB_CDC_ON_BOOT`).
  This is a *different board* from every other env in the repo (the S3s).
- **Driver:** BLD-120A "BLDC MOTOR DRIVER RUN/ALM", 12–30 V. Terminal order on
  the green block, left→right:
  `BRK EN F/R COM SV REF+ HU HV HW REF- W V U DC+ DC-`
- **Motor supply:** external **24 V** bench PSU on DC+/DC-, on its own. ESP on
  USB. **Never bridge the two grounds except through COM** (see wiring).
- **Second BLD-120A** sits on the same red 24 V rail but has **no control
  wires** → its EN floats high → it stays disabled. Harmless, but don't
  attribute a spin to the wrong driver.

### Verified wiring (colour map, confirmed from photo 2026-07-23)

The five control wires descend in a clean, non-crossing bundle in the same
order as the first five driver terminals:

| Wire colour | ESP pin | Driver terminal | Firmware constant |
|---|---|---|---|
| brown | GPIO18 | BRK (1) | `PIN_BRK` |
| red | GPIO17 | EN (2) | `PIN_EN` |
| orange | GPIO16 | F/R (3) | `PIN_FR` |
| black | GND | COM (4) | signal ground |
| white | GPIO25 | SV (5) | `PIN_SV` (DAC1) |

**Confidence:** the ESP end is Bryan's statement; the driver end is inferred
from wire *order* in the photo, not from seeing each conductor seated in its
screw terminal. **A continuity beep from each ESP header pin to its driver
terminal is the cheap way to remove all doubt** — see Task 1.

- **GPIO25 = DAC1.** Firmware defaults to true analog DC out (`dacWrite`), not
  PWM. `SV_USE_DAC 0` switches it back to 1 kHz LEDC PWM (matches production).
- GPIO16/17 are free here because WROOM-32D has **no PSRAM** (they'd be
  reserved on a WROVER/S3). GPIO2 (onboard LED heartbeat) is a strapping pin —
  fine to drive, just never hang a pull-up off it.

---

## Firmware behaviour (`src/bench_motor.cpp`)

- **Console library:** `ppedro74/SerialCommands` (already a repo dependency —
  library-first, no hand-rolled parser). One-key commands dispatch on the FIRST
  character, **before any Enter**, so the `x` E-STOP fires on keypress. This is
  the whole reason for the library and matters on a spinning motor.
- **Commands:**
  - instant (no Enter): `e`/`d` enable/disable · `f`/`r` fwd/rev ·
    `b`/`n` brake on/off · `+`/`-` nudge ±8 · `x` E-STOP · `?` help+status
  - with Enter: `s <0-255>` set command · `v <volts>` set by SV volts ·
    `sweep` auto-ramp 0→max→0
- **Boots SAFE:** disabled, forward, no brake, command 0.
- **Safety interlock (verified):** SV is gated on `(g_enabled && !g_brake)` in
  the control tick. A stale `s 60` cannot leak onto the driver until `e`.
  Confirmed on hardware: `s 60` set target 60/255 but status kept showing
  `cmd 0/255 → 0.00V` while disabled.
- **Slew limiter:** `SLEW_STEP` units/tick soft-ramps SV; `x` collapses it to 0
  immediately (no ramp-down on E-STOP).
- **Control lines are OUTPUT_OPEN_DRAIN** by default: asserting = pull to COM,
  releasing = Hi-Z (driver's internal pull-up holds high). `PUSH_PULL_CTRL 1`
  switches to plain 3.3 V push-pull.
- **Polarity constants carry Bryan's CONFIRMED-2026-07-15 values**, unverified
  by the agent this session: `EN_ASSERT=LOW`, `FR_FWD=HIGH`, `BRK_ASSERT=LOW`.
  Flip the relevant constant if a line behaves inverted.

---

## KNOWN LIMITATION — the 3.3 V ceiling (unresolved)

SV wants **0–5 V**; the ESP sources only **0–3.3 V**. Worse, SV is a pot-wiper
input that loads the source down: bench-measured **2.59 V at full command**
(2026-07-15) ≈ **~52 %** of the driver's 0–5 V span. **This rig therefore
cannot reach top speed.** It is fine for validating polarity / ramp / brake,
not for characterising the motor's top end.

**Library-first fix when full span is needed:**
- an **MCP4725** I2C DAC run off 5 V (`adafruit/Adafruit MCP4725`), or
- a non-inverting op-amp ×1.5 buffer on GPIO25.

A `HAS_SV_SENSE` block (default off) is already wired in the firmware: SV
through a 10 k/10 k divider into GPIO34 (ADC1, input-only, 5 V-safe) so you can
print what the driver ACTUALLY sees vs what was commanded.

---

## Open safety items (do these BEFORE enabling)

1. **THERE IS NO CURRENT PROTECTION AND NO SOFTWARE FAULT TRIP.**
   *Corrected 2026-07-27 — an earlier version of this document told you to set a
   ~1 A PSU current limit and called it "the only safety net". **The bench PSU has
   no current limit at all.** Do not rely on that instruction.*

   The BLD-120A has **no ALM output / no FG** (the "RUN/ALM" and "Peak Power" /
   "P-sv" markings on the case top are silkscreen *artwork*, not readable pot
   positions). The actual protection chain is: the driver's "Peak Power" trim
   (≤8 A, position unverified within its 0.8–8.0 A range), then a **10 A fuse**
   to the driver. Nothing else.

   Free-running at no load (~0.2–1 A) is not the concern. **Stall is:** a locked
   rotor held at the driver's limit sinks up to 8 A at 24 V = **192 W**, nearly
   all of it into stationary windings. The 10 A fuse cannot help — an 8 A stall
   sits below it indefinitely, so it never opens while the motor cooks. The fuse
   protects the cable, not the load.

   Mitigation is now software: the AS5600 added 2026-07-27 makes a genuine stall
   trip possible for the first time (command above break-away + RPM at zero →
   cut SV, brake, abort). See
   `docs/superpowers/specs/2026-07-27-bld120a-motor-calibration-design.md`.
   Until that ships, **do not leave the rig running unattended.**
2. **EN/F-R/BRK idle voltage unverified.** The open-drain config assumes the
   driver's internal pull-up idles at 3.3 V-safe levels. **With 24 V on and the
   ESP unplugged, probe EN / F/R / BRK against COM.** If they idle at **5 V**,
   they back-feed the ESP's clamp diodes on release (survivable ~0.1 mA, but out
   of spec) → add level shifting rather than trusting the clamps.

---

## Next steps (task list)

- [ ] **Task 1 — Verify wiring.** Continuity-beep each ESP header pin
      (GPIO18/17/16/GND/25) to its driver terminal (BRK/EN/F-R/COM/SV). Confirms
      the inferred colour map and rules out a brown↔red swap (which would make
      `e` brake and `b` enable).
- [ ] **Task 2 — Probe idle levels** (safety item 2 above).
- [x] **Task 3 — First spin.** DONE 2026-07-27: wheel turned and braked. (The
      "set PSU current limit LOW" precondition was void — there is no PSU current
      limit; see safety item 1.) Sequence used:
      `pio device monitor -e bench_motor` →
      `?` (confirm `EN:off cmd:0/255`) → `f` → `e` (motor live) → `s 60`
      (~0.78 V, should just break away) → `+` to nudge up → `x` to stop.
      **If `s 60` does nothing, suspect the driver's onboard P-sv trim is turned
      down — rule that out before blaming the 3.3 V ceiling.**
- [ ] **Task 4 — Confirm/flip polarity constants** (`EN_ASSERT`, `FR_FWD`,
      `BRK_ASSERT`) against observed behaviour; update the CONFIRMED date.
- [ ] **Task 5 (optional) — Recover full SV span** via MCP4725 or op-amp buffer
      (see Known Limitation), then enable `HAS_SV_SENSE` to close the loop on
      commanded-vs-measured SV.
- [ ] **Task 6 — Cleanup.** Once `src/bench_motor.cpp` is trusted, delete the
      superseded `bench_motor_test/` sketch and commit both new files.

---

## Provenance / prior facts pulled in

- Memory `falali-bld120a-driver`: real driver has NO FG/ALM; COM = signal
  gnd; F/R high = fwd; EN low = enable; 12–30 V.
- Memory `falali-bench-ble-gamepad`: 12 V killed an ESP once → the ~7.5 V
  rail note is for the *L298N mecanum* rig, NOT this BLD-120A rig (this one is
  correctly on its own 24 V PSU with isolated grounds).
- Bench log 2026-07-15: proportional ramp 0.78→2.59 V SV, F/R reversal, EN
  enable all validated on hardware — this firmware is a cleaner re-implementation
  of that same known-good behaviour.
