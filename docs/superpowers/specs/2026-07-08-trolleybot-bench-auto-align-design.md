# TrolleyBot — Bench auto edge-align (2-sensor) Design

**Date:** 2026-07-08
**Status:** Approved (brainstorm). Bench-validation increment, not production firmware.
**Owner:** Bryan
**Relates to:** `2026-07-06-trolleybot-esp32-docking-design.md` (the 4-corner production design this de-risks).

## 1. Goal & Scope

Validate the **sense-edge → rotate-to-orient** alignment concept on the real bench rig before
committing to the full 4-sensor hexagonal integration. Built into the standalone `src/bench_check.cpp`
(env `bench`), **not** the production `lib/` firmware.

**Rig:** ESP32-S3 · 2× VL53L0X (front-left = A, front-right = B, facing up) · 4 mecanum wheels on
2× L298N · PWM enables wired to GPIO4/5 (board A) and GPIO6/7 (board B).

**In scope:** interactive WASD mecanum jog; automatic ORIENT (Milestone 1); odometry-centred CENTER_X
(Milestone 2). **Out of scope:** CENTER_Y and the 4-corner CONFIRM (need the rear pair — later), the
clamp/arm subsystem, and the production `DockingStateMachine` integration.

## 2. Sensor placement

Both sensors on the **front (leading) edge**, one left (A), one right (B), facing up. They play the
`FL`/`FR` front pair from the production design. Address split: mux (ch0/ch1) if present, else XSHUT
(A@0x30, B@0x29) — auto-detected.

## 3. Behaviour

A `Mode` flag layered over the existing jog:

```
MANUAL ──(a front sensor's reading enters the height band: edge sensed)──▶ ORIENT
   ▲                                                                          │
   │  x (abort) / any WASD key (take manual control) / timeout / board lost   │
   └──────────────────────────────────────────────────────────────────────── │
                                                                              ▼
                                              ORIENT: rotate toward the lagging
                                              sensor until BOTH read in-band → stop
                                                                              │
                                                                              ▼
                                                                          ALIGNED
```

- **MANUAL:** `w/a/s/d` fwd/left/back/right, `q/e` rotate CCW/CW, space = stop, `+/-` jog speed,
  `1–4` single-wheel pulse (wiring check). Set-and-hold (motion persists until changed).
- **Auto-trigger:** on the **rising edge** of "either sensor in-band" while in MANUAL (so it fires as
  you drive under the trolley). `g` starts ORIENT manually; `t` toggles auto-trigger.
- **ORIENT:** rotate toward the lagging sensor (`w = A&&!B ? −1 : +1`, sign verified on hardware) at a
  gentle fixed duty until both are in-band, then stop → ALIGNED. Purely sensor-driven, no odometry.
- **CENTER_X (Milestone 2):** creep forward until both sensors leave the band (far edge), integrate
  commanded-velocity odometry, drive back to the **midpoint** between near/far edge events. Robust
  without speed calibration — the scale cancels because both marks use the same integrator.

## 4. Reused logic (mirrors the unit-tested domain)

- **Board-present** = `valid && band_min ≤ mm ≤ band_max`, with an N-sample debounce — same rule as
  `CornerEdgeDetector` / `CornerConfig` (defaults band 20–500 mm, debounce 2). Written inline for the
  2-sensor case; not throwaway thinking — it graduates to the real `DockingStateMachine` at 4 sensors.
- **ORIENT rotate sign** mirrors `DockingStateMachine::Orient`.
- **CENTER_X midpoint** mirrors `DockingStateMachine::CenterX`.

## 5. Safety

Auto-align uses a gentle fixed duty (well under manual full-speed); every auto phase has a timeout →
abort to MANUAL; `x` aborts anytime; both-sensors-lost for >1.5 s aborts; motors default stopped, and
abort/stop always drives all wheels to 0.

## 6. Interaction (tooling)

`pio device monitor` is broken on this host (miniterm `tcsetattr: Invalid argument`). A small
`teleop.py` (raw-stdin keypress → serial, background reader prints ToF/state) provides live WASD
control and replaces the monitor.

## 7. Verification

1. **PWM sanity** — set a low jog duty (`-`) and confirm a wheel visibly runs slower (proves the ENA/ENB
   caps are off and GPIO has sole enable control).
2. **Milestone 1 (ORIENT)** — hold a board over one sensor → platform rotates → stops when both are
   covered. Verify the rotate direction is *toward* the lagging sensor; flip the sign if not.
3. **Milestone 2 (CENTER_X)** — after orient, it creeps to the far edge and returns to the midpoint;
   the platform ends roughly centred along the travel axis over the board.

## 8. Open items

- Bench-tune the height band (`band_min/max`) to the real mounting height.
- Confirm the ORIENT rotate-direction sign and the mecanum strafe/rotate signs on hardware.
- Milestone 2 odometry integration (commanded-velocity) once ORIENT is trusted.
