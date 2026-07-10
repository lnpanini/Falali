# TrolleyBot — Bench BLE Gamepad Control Design

**Date:** 2026-07-10
**Status:** Approved (brainstorm). Bench-input increment, layered on the existing `src/bench_check.cpp`.
**Owner:** Bryan
**Relates to:** `2026-07-08-trolleybot-bench-auto-align-design.md` (the WASD + auto-align sketch this adds an input source to).

## 1. Goal & Scope

Add a **Bluetooth gamepad** as a manual-drive input for the bench rig, so the mecanum car can be
driven untethered (forward/back/strafe/rotate, proportionally) with an Xbox controller — while keeping
the existing serial WASD jog and the automatic edge-align behaviour fully intact.

**In scope:** proportional analog drive from an Xbox BLE controller via Bluepad32; button-triggered
auto-align start/abort; safe coexistence with serial WASD; fail-safe on link loss.
**Out of scope:** changes to the auto-align state machine (ORIENT / CENTER_X), the ToF sensing, the
production `DockingStateMachine`, and any non-BLE controller support.

## 2. Hardware constraint (the decision driver)

The **ESP32-S3 has Bluetooth Low Energy only — no Bluetooth Classic (BR/EDR) radio.** Consequences:

- **Nintendo Switch Pro Controllers / Joy-Cons use Bluetooth Classic HID → cannot pair with the S3.**
  No library can work around a radio that isn't present.
- **Modern Xbox Wireless Controllers (Bluetooth models, ~2016+) use BLE → these are the target device.**
  Some older Xbox One pads need a one-time firmware update (via a Windows PC / Xbox app) before they do
  standard BLE pairing; newer Series-model pads work out of the box.
- **Bluepad32** is the library: it supports gamepads over BTstack, and on the S3 (BLE-only) it drives
  BLE controllers such as the Xbox pad.

## 3. Architecture — build & code structure

**One source file, compile-flag gated, new PlatformIO env. The working `bench` env is left untouched.**

- All gamepad code lives in `src/bench_check.cpp` behind `#if defined(USE_GAMEPAD)`. The drivetrain,
  ToF sensing, and the entire auto-align state machine are shared verbatim between builds — one source
  of truth, no duplication.
- **`[env:bench]`** (existing) builds the file *without* the flag: serial-only, the proven fallback.
- **`[env:benchpad]`** (new) builds the *same* file *with* `-DUSE_GAMEPAD`, and swaps in Bluepad32's
  patched Arduino-ESP32 framework (Bluepad32 replaces the default Bluedroid stack with BTstack, so it
  ships its own framework build — it is **not** a plain `lib_deps` add-on). Keeps the same
  `board_build.arduino.memory_type = qio_opi`, the native-USB CDC flags
  (`ARDUINO_USB_CDC_ON_BOOT=1`, `ARDUINO_USB_MODE=1`), and `build_src_filter = +<bench_check.cpp>`.

> The exact Bluepad32 PlatformIO incantation (framework `platform_packages` URL and lib reference) is
> version-sensitive and will be pinned from Bluepad32's own docs during the implementation-plan step.

## 4. Control scheme — proportional analog

Bluepad32 is polled each `loop()`; stick values map to a drive intent, scaled and sent to the drive mix.

- **Left stick → translation (proportional):** `vx = −stickY` (up = forward), `vy = +stickX`
  (right = strafe right).
- **Right stick X → rotation:** `w = +stickX` (right = rotate CW).
- **Deadzone (~12%)** on each axis to reject stick drift, and a **minimum-duty floor** so small
  deflections still overcome motor stiction (map magnitude above the deadzone into
  `[MIN_MOVE_DUTY, 255]`).
- **Buttons:** **A** → start auto-align (equivalent to serial `g`); **B** → abort to MANUAL
  (equivalent to serial `x`), both on button rising-edge.
- Any stick motion sets `Mode::Manual`, taking control back from auto-align exactly as a WASD key does.
- **Sign conventions** (stick-axis → `vx/vy/w`, rotate direction) are **verify-on-hardware**, mirroring
  the existing WASD / ORIENT conventions in the sketch.

### 4.1 Proportional drive mix (the one refactor)

Add `driveMixF(float vx, float vy, float w, int spd)` computing the four wheel sums and **normalising by
the maximum magnitude** so blended translate+rotate commands scale together instead of clipping
independently:

```
fl = vx+vy+w   fr = vx−vy−w   rl = vx−vy+w   rr = vx+vy−w
m  = max(1, max(|fl|,|fr|,|rl|,|rr|))
wheel_i = (sum_i / m) * spd
```

The existing `driveMix(int,int,int,int)` becomes a thin wrapper over `driveMixF`. Its single-axis calls
from the state machine — `(1,0,0)`, `(0,0,±1)`, `(dir,0,0)` — produce identical output, so **the ORIENT
/ CENTER_X logic needs no changes.**

## 5. Coexistence & untether-safety

- **Gamepad and serial WASD are two input sources into the same `driveMix`; last input wins.** Wired,
  both are live; untethered, the gamepad drives alone.
- **Untether-safety:** on this board `Serial` is the native USB CDC. Writing to USB CDC with **no host
  connected can block** the loop (TX FIFO fills, write waits on a timeout) — and Bluepad32 ships its own
  core build, so the safe-vs-blocking behaviour can't be assumed. Therefore **every `Serial` write
  (heartbeat + edge events) is guarded with `if (Serial) { … }`**, so unplugging USB to go wireless
  never stalls the control loop. The read side is already safe (`Serial.available()` returns 0 with no
  host).

## 6. Fail-safe (link loss)

Manual drive from a wireless link must fail **stopped**, never latched at the last stick value:

- **Bluepad32 disconnect callback → `stopAll()`** and zero the drive intent.
- **Watchdog:** if the gamepad is the active input source and no fresh update arrives within **~300 ms**,
  `stopAll()`. This covers a BLE *stall* (out of range, controller sleeps, battery dies) that never
  fires a clean disconnect.
- Motors default stopped; every abort path (serial `x`, auto-align timeouts, board-lost, disconnect,
  watchdog) drives all wheels to 0. The auto-align phase timeouts from the prior design are unchanged.

## 7. Reused / unchanged

- **Auto-align state machine** (MANUAL → ORIENT → CENTER_X → CENTERED), ToF bring-up (mux/XSHUT
  auto-detect), presence debounce, odometry-centred CENTER_X — all as in the bench auto-align design.
- **Serial WASD/CAL keymap** — unchanged; still the wired control and calibration path.

## 8. Verification (hardware-first)

1. **Pair** the Xbox pad (hold the pair button; Bluepad32 scans and connects). Confirm connect/disconnect
   log lines.
2. **Axis check:** each left-stick direction drives the correct wheels (fwd/back/strafe); right-stick X
   rotates the correct way; centre stick holds still (deadzone). Flip any sign that's reversed.
3. **Proportional check:** partial stick = visibly slower than full stick.
4. **Buttons:** **A** starts ORIENT; **B** aborts to MANUAL.
5. **Fail-safe:** power the controller **off mid-drive** → motors stop within the watchdog window
   (~300 ms). Repeat by walking it out of BLE range.
6. **Untether:** unplug USB while driving on the gamepad → control loop keeps running smoothly (no stall
   from guarded serial writes).
7. **Regression:** `pio run -e bench -t upload` (serial-only build) still drives via WASD and auto-aligns.
8. **Optional host guardrail:** a pure-math check of `driveMixF` normalisation (no hardware).

## 9. Open items / risks

- Pin the exact Bluepad32 PlatformIO framework/lib references from its current docs (implementation-plan
  step).
- Confirm the specific Xbox pad's BLE pairing (firmware-update needed for older Xbox One models).
- Confirm stick-axis and rotate **sign conventions** on hardware.
- Bench-tune the stick **deadzone** and **`MIN_MOVE_DUTY`** to the actual controller and motors.
- Verify the guarded-serial approach holds on Bluepad32's core build (no residual blocking when USB is
  disconnected).
