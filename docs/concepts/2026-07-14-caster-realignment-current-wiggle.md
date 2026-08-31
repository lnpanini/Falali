# Concept: Current-Sensed Caster Re-Alignment ("detect-and-wiggle")

**Status:** PARKED CONCEPT — not scheduled, not designed against the codebase yet.
**Date:** 2026-07-14
**Related:** [caster load mechanics report](../research/2026-07-13-caster-load-mechanics.md)

## Problem it addresses

A trolley on 4 swivel casters resists motion most when its casters are **misaligned** (pointing across the intended travel direction after a stop or reversal). Reorienting a *static* caster costs ~10× the force of a *rolling* one, producing a startup spike of roughly **150–260 N for a 200 kg load** (~10–13 % of load weight). This spike, not steady rolling, is what defeats a light pusher.

## The idea

1. **Detect** elevated resistance via **drive-motor current sensing** (misaligned caster → higher resistance → higher current). Mirrors the existing clamp stall-current pattern (`kClampStallAmps`).
2. **Correct** by superimposing a small **yaw/lateral "wiggle"** on forward creep — keep the casters micro-rolling so they reorient in the cheap rolling regime instead of static scrub. Because the robot clamps under the trolley *centre* and the casters are at the corners, small yaw gives the caster pivots real velocity (`v = ω × r`), so the leverage is favourable.
3. **Confirm** alignment when current drops back to the rolling baseline, then resume straight travel.

## Why it works (physics)

- Rolling reorientation ≈ **1/10th** the force of static scrub (relaxation-length behaviour).
- Never letting the system go fully static avoids the breakaway/misaligned-start spike entirely.
- Confirmed by the ergonomics literature: reversed/misaligned caster starts measurably raise initial push force, worse for larger casters (IEEE IEEM 2015; ScienceDirect S0003687002000029).

## CRITICAL caveat — complement, not substitute

This addresses the **caster-flip** problem, **not the traction ceiling**. If the robot is traction-limited (light robot, no weight transfer, 100–200 kg load), the wheels **slip before** the current signal is actionable — and on slip, mecanum current *falls*, so current sensing detects "already stuck," not "about to stick." **Order of operations:** first ensure adequate traction (robot mass and/or weight transfer / lift — see the mechanics report), *then* detect-and-wiggle earns its keep as a caster-flip handler. It cannot rescue a traction deficit.

## Where it sits among alternatives (cheapest → most robust)

- **Open-loop back-up jog** (reverse 50–100 mm before a stroke): aligns all casters proactively, no sensing, no mid-stroke disturbance. Do this first.
- **This concept (detect-and-wiggle):** the closed-loop *reactive* fallback for a caster that stalls mid-path (e.g. right after a turn) despite the jog.
- **Lift the casters:** unloaded casters self-home — removes the problem entirely (Amazon/Kiva patent US 12,472,772).

Recommended layering: **jog proactively → wiggle reactively → all gated on having the traction to actuate either.**

## Sensing requirements (resolved 2026-07-14)

- **Detection:** drive-motor **current sense** only — no shaft position needed.
- **Wiggle execution:** small current-gated dither; needs at most coarse speed feedback.
- **Speed feedback:** use existing **BLDC hall sensors** + the **BLD120A `FG` output**; apply **period-measurement (time between hall edges)** to keep resolution usable at the low creep speeds where the wiggle runs.
- **Heading / drift correction:** add an **IMU** — for a mecanum robot dragging ~100 kg, wheel odometry is corrupted by roller/load slip and an IMU is the right heading source.
  - **Recommended part (2026-07-15, not yet purchased):** a modern **6-axis** gyro/accel — *not* a 9-axis. The magnetometer that would give absolute heading is unreliable inches from 4 BLDC motors (switching currents + steel swamp Earth's tiny field), so a 9-axis buys a mag axis you can't trust.
  - **Architecture:** gyro for smooth short-term yaw (holds heading through a maneuver) **+ the dock ToF as the periodic absolute correction**. That makes the mag redundant.
  - **AliExpress candidates:** **ICM-42688-P** (~$5, low-noise gyro — best value pick); **MPU6050** (~$1.50, drifts over minutes but fine to prototype); or **BNO085 in UART-RVC mode** (~$12–20, on-chip fusion streams yaw/pitch/roll → zero fusion code). Watch counterfeits on MPU6050/9250 boards.
  - **Placement matters more than the chip:** mount far from motors/high-current wiring, near the rotation centre, on foam/rubber for vibration damping.
- ~~**Encoders (AS5600) — DEFERRED.**~~ **ABANDONED.** Written as deferred; they were never made to work and the project shipped without wheel-speed feedback of any kind. The AS5600 boards and shaft magnets are **still physically mounted on the motors**, unwired and absent from the firmware. Two board-level faults killed them — see [`docs/hardware/boards.md`](../hardware/boards.md) §1. The TCA9548A mux route suggested below went with them; if anyone revisits wheel-speed feedback, **the motors' own hall sensors land on the drivers' screw terminals** and are the cheaper route ([`docs/hardware/components.md`](../hardware/components.md) §2).

## Open questions / next steps to prototype

1. Bench-measure the actual current signature of a deliberately-misaligned caster vs baseline — does it clear the mecanum roller-hand-off noise floor?
2. Verify traction margin exists first (robot mass / weight transfer) so the signal is actionable.
3. Tune wiggle amplitude/frequency for reliable alignment with minimum load/odometry disturbance.
4. Decide the detect→wiggle→confirm loop as a state added to / gated by `DockingStateMachine` + `SafetyMonitor`.
5. Regardless of this concept: current sensing has standing value as a **slip/jam/obstacle monitor** feeding `SafetyMonitor`.

*When promoted from concept to feature: run a proper brainstorm and graphify-ground it against the existing `DockingStateMachine` / `SafetyMonitor` structure before implementing.*
