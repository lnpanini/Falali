# BLD-120A + AS5600 Motor Calibration — Design

**Date:** 2026-07-27
**Rig:** ESP32-WROOM-32D + BLD-120A driver + BLDC motor + 15:1 gearbox, AS5600 on the
motor output shaft. Bench PSU 24 V, **no current limiting** — see Safety.
**Firmware:** `src/bench_motor.cpp`, `[env:bench_motor]`.

---

## Established facts (measured 2026-07-27, not assumptions)

| Quantity | Value | How known |
|---|---|---|
| SV → motor RPM | **1012 RPM/V**, linear | measured at cmd 60 / 100 / 160 |
| Rolloff at cmd 160 | ~1.3 % below the low-end fit | measured |
| Encoder | AS5600 @ 0x36, 4096 cpr, GPIO21/22 | I2C scan |
| Encoder sampling | 250 Hz (4 ms), 400 kHz I2C | firmware |
| Sampling ceiling | 7500 motor RPM before wrap aliasing | 0.5 rev/sample × 250 Hz |
| ESP DAC ceiling | 3.3 V; droops to ~2.59 V loaded | bench log 2026-07-15 |
| Predicted top speed | ~2621 motor RPM at 2.59 V | 1012 × 2.59 |
| Gearbox | **15:1**, known ratio | user-supplied |
| Motor counts / wheel rev | **61 440** (15 × 4096) | derived |
| Wheel RPM per SV volt | **67.5** | 1012 / 15 |
| Brake stop | within one 500 ms status window | observed |
| Driver feedback | none — no FG, no ALM | BLD-120A has no such outputs |

## Goals

1. **Transfer curve** — deadband/break-away, linear slope, saturation knee, stiction
   hysteresis, and symmetry between the two F/R states.
2. **Step response** — rise time constant, coast-down, brake-down. Inputs for the
   deferred `br3ttb/PID` dependency and for predicting docking stopping distance.
3. **Gearbox verification** — confirm the 15:1 label against reality.

## Non-goals

- **Mapping F/R to robot-forward.** The motor is secured to a test bench and its
  orientation in the chassis is not confirmed. All direction results here are
  **driver-frame** (`FR=HIGH` / `FR=LOW`), never labelled forward/reverse.
  Resolving this is a one-line check after chassis mounting: command `FR=HIGH`,
  observe which way the robot moves.
- Closed-loop control. This produces the constants; the controller is separate work.
- Recovering the full 0–5 V SV span (MCP4725 / op-amp buffer). Deferred; the
  measured curve will quantify what the 3.3 V ceiling actually costs.
- Multi-motor. Single motor rig; the AS5600's fixed 0x36 address blocks 4 encoders
  on one bus regardless (needs TCA9548A or per-bus wiring).

---

## Architecture

**Firmware owns timing; the host owns analysis.**

The firmware runs each procedure as a **non-blocking state machine driven from
`loop()`** and emits machine-readable CSV to the console. Analysis (fitting,
plotting) happens on the host, or in a spreadsheet directly from the CSV.

### Why non-blocking is a hard requirement, not a style preference

A full sweep runs ~2.6 minutes. During any blocking `delay()` loop,
`g_cli.ReadSerial()` never runs, so **`x` stops working**. An e-stop that is dead
for minutes, on a motor secured to a bench spinning near 2600 RPM, is a worse
outcome than having no calibration at all. Every dwell is a `millis()` comparison.
This matches the existing `sweep` command's structure.

### Invariants every routine must hold

- `x` aborts at any point, from any state, and lands in the existing e-stop state.
- The existing `(g_enabled && !g_brake)` SV interlock is never bypassed.
- The slew limiter stays in force. With no PSU current limit it no longer protects
  the supply — it now exists to keep inrush off the driver and to stop a command
  step from looking like a stall to the trip below.
- **Stall detection is armed for the entire run** (see Safety). No routine may
  disable it, including during the deliberate stop captures in Phase 2.
- On abort or completion, the routine restores command 0 and leaves the rig safe.
- CSV rows are prefixed so they can be grepped out of the interleaved status stream.

---

## Phase 1 — Transfer curve (`calsweep`)

Steps the command in increments of 8 across four legs:

| Leg | F/R state | Direction |
|---|---|---|
| 1 | `FR=HIGH` | 0 → 255 |
| 2 | `FR=HIGH` | 255 → 0 |
| 3 | `FR=LOW` | 0 → 255 |
| 4 | `FR=LOW` | 255 → 0 |

At each of the 33 points per leg: settle **800 ms**, then average RPM over **400 ms**.
Measured settling was under one 500 ms status window, so 800 ms carries margin.
Total runtime ≈ **158 s**.

### Why up *and* down

The down-sweep yields stiction hysteresis for free: break-away command on the way up
will exceed drop-out on the way down. That gap is the quantity the `KICK_DUTY` /
`KICK_MS` anti-stall constants in `src/bench_align.h` currently guess at.

### Why both F/R states

Catches driver asymmetry, and independently verifies that the **encoder sign flips
with F/R**. If both states report the same sign, the sign convention is broken and
every downstream odometry integration would accumulate in the wrong direction.

### Output

```
CSV,sweep,fr_state,leg_dir,cmd,sv_volts,rpm_motor,rpm_wheel,counts_delta,agc,mag_status
```

`agc` and `mag_status` are logged **per point, not once at the start**. A weak or
distant magnet fails intermittently, and a dropout partway through a 158 s leg would
otherwise be indistinguishable from a real measurement. Logging them per row turns a
silent corruption into a visible one. If `MD` clears or `ML` sets mid-run, the routine
aborts the leg and says which command it happened at, rather than finishing and
handing back a plausible-looking curve with a hole in it.

`fr_state` is `HIGH`/`LOW`. `rpm_wheel` = `rpm_motor / 15`. Sign of `rpm_motor` is
recorded as measured, never forced.

### Derived results

- Break-away command per F/R state — first cmd where **|RPM| > 5 motor RPM**.
  Steady-state readings hold to ±0.5 RPM, so 5 RPM is ~10× the observed noise.
- Drop-out command per F/R state — on the down-leg, first cmd where |RPM| falls
  back below that same 5 RPM floor
- Hysteresis = break-away − drop-out
- Linear-region slope (RPM/V) and intercept, fitted over the linear span
- Saturation knee: the command where measured RPM departs the linear fit by >5 %
- Symmetry: |slope(HIGH)| vs |slope(LOW)|

---

## Phase 2 — Step response (`calstep <cmd>`)

Runs after the curve, which supplies the linear region needed to pick a sensible
step size. Logs at the full 250 Hz sample rate, not the 2 Hz status rate. Three
captures:

1. **Rise** — step 0 → cmd, log ~2 s. Yields time constant τ (63 % of final),
   rise time, overshoot, steady-state value.
2. **Coast** — from steady speed, disable with brake **off**, log until stopped.
   Pure mechanical friction; tells you how far the rig freewheels.
3. **Brake** — from steady speed, assert brake, log until stopped. The difference
   from coast is the brake's actual contribution.

### Output

```
CSV,step,phase,t_ms,cmd,rpm_motor
```

`phase` is `rise` / `coast` / `brake`. `t_ms` is relative to the start of that capture.

---

## Phase 3 — Gearbox verification (`calgear`)

**Method: powered run, user counts wheel revolutions.**

Signature: `calgear [cmd]`, defaulting to **cmd 60** (known to break away reliably;
789 motor RPM = 52.6 wheel RPM = 0.88 rev/s). If Phase 1 has run, prefer
**measured break-away + 10** instead — the slowest reliable speed makes the
revolutions easiest to count by eye.

1. Zero the encoder (`z`).
2. Spin at `cmd` for a duration computed from the measured 1012 RPM/V curve to
   yield **~10 predicted wheel revolutions** (≈11 s at cmd 60). Predicted only —
   the result comes from what the user actually counts.
3. Brake to a stop, then wait for the encoder to go quiet.
4. Report **total motor counts**, including counts accumulated during coast — the
   encoder keeps counting through spin-down, so coast does not corrupt the result.
5. User reports observed wheel revolutions (mark the wheel first).
6. `measured_ratio = motor_counts / (4096 × wheel_revs)`, compared against 15.

Deliberately **not** "stop at exactly 61 440 counts": the shaft coasts after power is
cut, so the resting position would not equal the target and the error would be read
as gearbox error.

**Alternative if the gearbox backdrives:** with the motor unpowered, turn the wheel
by hand exactly N revolutions and read the counts. More accurate — no deadband, no
coast, no timing. Worm gearboxes generally cannot backdrive, hence the powered method
as primary.

### Output

```
CSV,gear,motor_counts,wheel_revs_reported,measured_ratio,label_ratio,error_pct
```

---

## Console additions

| Command | Enter? | Action |
|---|---|---|
| `calsweep` | yes | Phase 1, four legs, ~158 s |
| `calstep <cmd>` | yes | Phase 2 at the given command |
| `calgear [cmd]` | yes | Phase 3, default cmd 60 |
| `x` | no | aborts any of the above (existing e-stop) |

Existing commands are unchanged.

---

## Safety

- Motor is secured to a test bench; sweep runs the full 0–255 span by explicit
  user decision (2026-07-27).
- **THERE IS NO CURRENT PROTECTION.** Corrected 2026-07-27: an earlier note in this
  spec and in `docs/superpowers/plans/2026-07-25-bld120a-motor-bench-bringup.md`
  claimed a ~1 A PSU current limit was set. It is not. The BLD-120A also has no ALM
  output. The rig therefore has **no hardware fault protection of any kind**, and the
  driver's own "Peak Power" trim is at an unknown setting.
- Protection chain, worst case first: the driver's "Peak Power" trim (**≤8 A**,
  actual position unverified within its 0.8–8.0 A range) → a **10 A fuse** to the
  driver. Nothing else.
- Free-running at no load draws roughly 0.2–1 A and is not the concern. **Stall is.**
  Held at the driver's limit, a locked rotor sinks up to 8 A at 24 V = **192 W**,
  almost all of it into stationary windings. The 15:1 gearbox makes a wheel-side jam
  a hard stall at the motor.
- **The 10 A fuse cannot protect the motor**, by construction: an 8 A stall sits
  below the fuse rating indefinitely, so the fuse never opens while the windings
  cook. The fuse is sized for the cable, not the load. There is no current at which
  this arrangement both saves the motor and blows the fuse.

### Stall detection — mandatory, replaces the missing current limit

The AS5600 makes a genuine software fault trip possible for the first time; before
the encoder there was no feedback to trip on. Every calibration routine MUST run it:

- **Trip condition:** commanded SV above the measured break-away point AND
  |RPM| < 5 for longer than `STALL_TRIP_MS` (250 ms).
- **Response:** collapse SV to 0, assert brake, abort the routine, print a loud
  diagnostic naming the command at which it tripped.
- **Grace period:** suppressed during the first 300 ms after any command increase,
  so normal acceleration from rest is not read as a stall.
- Keys on the actual fault rather than on current, so unlike a fixed current limit it
  does not false-trip on legitimate inrush. Reacts in ~250 ms — faster than a human,
  and faster than a slow-blow fuse.

**Stall detection is a mitigation, not a substitute for fusing.** It cannot protect
against a driver-side short, where the MOSFETs fail closed and no command from the
ESP has any effect. An inline fuse sized just above the measured full-command running
current is the complement, and should be fitted before unattended runs.

- **Supply-sag detection:** if measured RPM *falls* while command *rises*, the supply
  is sagging or the driver is limiting internally. Flag it in the output rather than
  silently recording a bent curve as a real saturation knee.
- The `x` e-stop stays live throughout, guaranteed by the non-blocking design.
- Note: `x` latches the brake (`g_brake = true`) with no auto-clear, so `n` is
  required before the next run. Calibration routines must refuse to start while
  braked and say why, rather than appearing to hang.

---

## Testing

The pure arithmetic — linear fit, break-away/drop-out detection, hysteresis,
knee detection, ratio computation — is testable on the host without hardware and
belongs under `[env:native]` with the existing Unity setup, fed synthetic count
series. The state machines and I2C reads are hardware-verified on the bench.

Verification that the rig itself is honest: re-running `calsweep` should reproduce
the cmd 60 / 100 / 160 points already measured (789 / 1305 / 2067 motor RPM) within
noise. A mismatch means something changed since 2026-07-27, not that the curve is new.

---

## Open items

- Gearbox ratio 15:1 is user-supplied and verified only by Phase 3.
- Magnet airgap: AGC is currently railed at max gain (128/128 at 3.3 V), meaning the
  magnet sits too far from the chip. Tracking is stable now but has no margin.
  **Move the magnet closer (aim AGC ~64) before trusting calibration data**, since a
  dropout mid-sweep would corrupt a leg silently.
- F/R → robot-forward mapping deferred to chassis mounting (see Non-goals).
