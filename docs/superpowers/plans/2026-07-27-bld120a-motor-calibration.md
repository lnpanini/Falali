# BLD-120A Motor Calibration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add three calibration routines (`calsweep`, `calstep`, `calgear`) to the BLD-120A bench firmware, plus a mandatory stall trip, so the motor's SV→RPM curve, step response, and gearbox ratio become measured numbers.

**Architecture:** Analysis math and the stall trip go in `lib/domain/` as pure C++17 classes with host unit tests (matching the repo's existing hexagonal split). `src/bench_motor.cpp` gains three non-blocking state machines driven from `loop()` that own timing and I/O only. No `delay()` anywhere — a blocking loop would kill the `x` e-stop for minutes.

**Tech Stack:** PlatformIO, Arduino-ESP32 core 3.2.1, `ppedro74/SerialCommands`, `robtillaart/AS5600` 0.6.7, Unity (host tests via `[env:native]`).

Spec: `docs/superpowers/specs/2026-07-27-bld120a-motor-calibration-design.md`

## Global Constraints

- Target board is **ESP32-WROOM-32D** (`[env:bench_motor]`, `board = esp32dev`). Not the S3. I2C is **GPIO21/22**, never the 38/39 in `include/pins.h`.
- Gearbox ratio is **15:1**. Encoder is **4096 cpr** on the **motor output shaft**. Motor counts per wheel revolution = **61440**.
- Directions are **driver-frame only**: label `FR=HIGH` / `FR=LOW`. Never emit "forward"/"reverse" — chassis orientation is unconfirmed.
- **`x` must abort at any point, from any state.** Every routine is a state machine polled from `loop()`; `delay()` is forbidden in routine code.
- **Stall detection is armed for the entire run** and no routine may disable it, including during Phase 2's deliberate stop captures.
- Routines refuse to start while the brake is latched (`x` sets `g_brake` with no auto-clear) and must say why.
- Pure C++17 in `lib/domain/`: no Arduino headers, `namespace tb`, `#pragma once`, trailing-underscore privates.
- CSV rows are prefixed `CSV,` so they can be grepped out of the interleaved 2 Hz status stream.
- Host tests run with `pio test -e native`. Firmware builds with `pio run -e bench_motor`.

---

### Task 1: StallDetector (pure, host-tested)

Safety-critical and entirely time/threshold logic, so it is built and tested with zero hardware.

**Files:**
- Create: `lib/domain/StallDetector.h`
- Create: `lib/domain/StallDetector.cpp`
- Test: `test/test_stall_detector/test_stall_detector.cpp`

**Interfaces:**
- Consumes: nothing (first task)
- Produces: `tb::StallConfig{ int break_away_cmd; float rpm_floor; uint32_t trip_ms; uint32_t grace_ms; }`, `tb::StallDetector` with `void reset(uint32_t now_ms)`, `void noteCommandIncrease(uint32_t now_ms)`, `bool update(uint32_t now_ms, int cmd, float rpm)` returning true on the trip edge, `bool tripped() const`, `int trippedAtCmd() const`

- [ ] **Step 1: Write the failing test**

Create `test/test_stall_detector/test_stall_detector.cpp`:

```cpp
// Host tests for StallDetector — the software fault trip that replaces the
// bench's missing current limit. The BLD-120A has no ALM output, so this is
// the rig's only automatic protection against a locked rotor at 192 W.
#include <unity.h>

#include "StallDetector.h"

using namespace tb;

void setUp() {}
void tearDown() {}

static StallConfig cfg() {
  StallConfig c;
  c.break_away_cmd = 40;
  c.rpm_floor = 5.0f;
  c.trip_ms = 250;
  c.grace_ms = 300;
  return c;
}

void test_no_trip_below_break_away() {
  // Below break-away the motor is EXPECTED to be still. Zero RPM is not a fault.
  StallDetector d(cfg());
  d.reset(0);
  for (uint32_t t = 0; t <= 2000; t += 50) {
    TEST_ASSERT_FALSE(d.update(t, 20, 0.0f));
  }
  TEST_ASSERT_FALSE(d.tripped());
}

void test_trips_when_commanded_but_not_turning() {
  StallDetector d(cfg());
  d.reset(0);
  bool edge = false;
  for (uint32_t t = 0; t <= 1000; t += 50) {
    if (d.update(t, 100, 0.0f)) edge = true;
  }
  TEST_ASSERT_TRUE(edge);
  TEST_ASSERT_TRUE(d.tripped());
  TEST_ASSERT_EQUAL_INT(100, d.trippedAtCmd());
}

void test_no_trip_while_turning() {
  StallDetector d(cfg());
  d.reset(0);
  for (uint32_t t = 0; t <= 2000; t += 50) {
    TEST_ASSERT_FALSE(d.update(t, 100, 800.0f));
  }
}

void test_grace_period_suppresses_trip_after_command_increase() {
  // Accelerating from rest must not read as a stall.
  StallDetector d(cfg());
  d.reset(0);
  d.noteCommandIncrease(0);
  // 250ms of zero RPM would normally trip, but we are inside the 300ms grace.
  for (uint32_t t = 0; t <= 280; t += 20) {
    TEST_ASSERT_FALSE(d.update(t, 100, 0.0f));
  }
}

void test_trips_after_grace_expires() {
  StallDetector d(cfg());
  d.reset(0);
  d.noteCommandIncrease(0);
  bool edge = false;
  for (uint32_t t = 0; t <= 1200; t += 20) {
    if (d.update(t, 100, 0.0f)) edge = true;
  }
  TEST_ASSERT_TRUE(edge);
}

void test_negative_rpm_counts_as_turning() {
  // FR=LOW spins the other way; magnitude is what matters, not sign.
  StallDetector d(cfg());
  d.reset(0);
  for (uint32_t t = 0; t <= 2000; t += 50) {
    TEST_ASSERT_FALSE(d.update(t, 100, -800.0f));
  }
}

void test_trip_edge_fires_once() {
  StallDetector d(cfg());
  d.reset(0);
  int edges = 0;
  for (uint32_t t = 0; t <= 2000; t += 50) {
    if (d.update(t, 100, 0.0f)) edges++;
  }
  TEST_ASSERT_EQUAL_INT(1, edges);
}

void test_reset_clears_trip() {
  StallDetector d(cfg());
  d.reset(0);
  for (uint32_t t = 0; t <= 1000; t += 50) d.update(t, 100, 0.0f);
  TEST_ASSERT_TRUE(d.tripped());
  d.reset(2000);
  TEST_ASSERT_FALSE(d.tripped());
}

void test_recovery_restarts_the_window() {
  // Motor briefly turns, then stalls. The 250ms window restarts on motion,
  // so a momentary stall shorter than trip_ms must not accumulate.
  StallDetector d(cfg());
  d.reset(0);
  TEST_ASSERT_FALSE(d.update(0,   100, 0.0f));
  TEST_ASSERT_FALSE(d.update(200, 100, 0.0f));   // 200ms stalled, under 250
  TEST_ASSERT_FALSE(d.update(240, 100, 900.0f)); // turning again -> window resets
  TEST_ASSERT_FALSE(d.update(400, 100, 0.0f));   // only 160ms since restart
  TEST_ASSERT_FALSE(d.tripped());
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_no_trip_below_break_away);
  RUN_TEST(test_trips_when_commanded_but_not_turning);
  RUN_TEST(test_no_trip_while_turning);
  RUN_TEST(test_grace_period_suppresses_trip_after_command_increase);
  RUN_TEST(test_trips_after_grace_expires);
  RUN_TEST(test_negative_rpm_counts_as_turning);
  RUN_TEST(test_trip_edge_fires_once);
  RUN_TEST(test_reset_clears_trip);
  RUN_TEST(test_recovery_restarts_the_window);
  return UNITY_END();
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_stall_detector`
Expected: FAIL — compile error, `StallDetector.h` not found.

- [ ] **Step 3: Write minimal implementation**

Create `lib/domain/StallDetector.h`:

```cpp
// The bench rig's only automatic fault protection.
//
// The BLD-120A exposes no ALM output and no FG, and the bench PSU has no current
// limit — so before the AS5600 was fitted there was literally no signal from which
// a stall could be inferred. A locked rotor held at the driver's limit sinks up to
// 8 A at 24 V (~192 W) into stationary windings, and the 10 A fuse never opens
// because 8 A sits below it indefinitely.
//
// The trip keys on the actual fault — "commanded to move, not moving" — rather
// than on current, so it does not false-trip on legitimate inrush.
#pragma once

#include <stdint.h>

namespace tb {

struct StallConfig {
  // Below this command the motor is expected to be still, so zero RPM is normal
  // rather than a fault. Set from the measured break-away point.
  int      break_away_cmd = 40;
  float    rpm_floor      = 5.0f;   // |RPM| under this counts as "not turning"
  uint32_t trip_ms        = 250;    // sustained stall before tripping
  uint32_t grace_ms       = 300;    // suppression after a command increase
};

class StallDetector {
public:
  StallDetector() = default;
  explicit StallDetector(const StallConfig& cfg) : cfg_(cfg) {}

  // Clears the trip and restarts all timing. Call before every routine.
  void reset(uint32_t now_ms);

  // Opens a grace window. Call whenever the command steps UP, so ordinary
  // acceleration from rest is not mistaken for a locked rotor.
  void noteCommandIncrease(uint32_t now_ms);

  // Returns true ONCE, on the transition into the tripped state.
  bool update(uint32_t now_ms, int cmd, float rpm);

  bool tripped() const { return tripped_; }
  int  trippedAtCmd() const { return tripped_at_cmd_; }

private:
  StallConfig cfg_{};
  bool     tripped_        = false;
  int      tripped_at_cmd_ = 0;
  bool     stalling_       = false;  // currently inside a candidate stall
  uint32_t stall_since_    = 0;
  uint32_t grace_until_    = 0;
};

} // namespace tb
```

Create `lib/domain/StallDetector.cpp`:

```cpp
#include "StallDetector.h"

namespace tb {

static inline float absf(float v) { return v < 0.0f ? -v : v; }

void StallDetector::reset(uint32_t now_ms) {
  tripped_ = false;
  tripped_at_cmd_ = 0;
  stalling_ = false;
  stall_since_ = now_ms;
  grace_until_ = now_ms;
}

void StallDetector::noteCommandIncrease(uint32_t now_ms) {
  grace_until_ = now_ms + cfg_.grace_ms;
  stalling_ = false;  // restart the window; the motor is being asked to speed up
}

bool StallDetector::update(uint32_t now_ms, int cmd, float rpm) {
  if (tripped_) return false;  // edge already delivered

  // Not commanded hard enough to expect motion -> nothing to police.
  if (cmd < cfg_.break_away_cmd) { stalling_ = false; return false; }

  // Inside the post-step grace window -> acceleration, not a stall.
  if ((int32_t)(now_ms - grace_until_) < 0) { stalling_ = false; return false; }

  const bool moving = absf(rpm) >= cfg_.rpm_floor;
  if (moving) { stalling_ = false; return false; }

  if (!stalling_) { stalling_ = true; stall_since_ = now_ms; return false; }

  if (now_ms - stall_since_ >= cfg_.trip_ms) {
    tripped_ = true;
    tripped_at_cmd_ = cmd;
    return true;
  }
  return false;
}

} // namespace tb
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pio test -e native -f test_stall_detector`
Expected: PASS, 9 tests.

- [ ] **Step 5: Commit**

```bash
git add lib/domain/StallDetector.h lib/domain/StallDetector.cpp test/test_stall_detector/test_stall_detector.cpp
git commit -m "feat(bench): add StallDetector — software fault trip for the BLD-120A rig"
```

---

### Task 2: MotorCalAnalysis (pure, host-tested)

All the curve arithmetic. Kept out of the firmware so it can be tested against synthetic data with known answers.

**Files:**
- Create: `lib/domain/MotorCalAnalysis.h`
- Create: `lib/domain/MotorCalAnalysis.cpp`
- Test: `test/test_motor_cal_analysis/test_motor_cal_analysis.cpp`

**Interfaces:**
- Consumes: nothing
- Produces: `tb::CalPoint{ int cmd; float sv_volts; float rpm; }`, `tb::LinearFit{ float slope_rpm_per_volt; float intercept_rpm; bool valid; }`, and free functions `LinearFit fitLinear(const CalPoint*, size_t)`, `int breakAwayCmd(const CalPoint*, size_t, float)`, `int dropOutCmd(const CalPoint*, size_t, float)`, `int kneeCmd(const CalPoint*, size_t, const LinearFit&, float, float)`, `float gearRatio(int32_t, float, int32_t)`

- [ ] **Step 1: Write the failing test**

Create `test/test_motor_cal_analysis/test_motor_cal_analysis.cpp`:

```cpp
// Host tests for the motor calibration arithmetic, fed synthetic curves with
// known answers. The bench measured 1012 RPM/V on 2026-07-27, so that slope is
// used as the realistic case throughout.
#include <unity.h>

#include "MotorCalAnalysis.h"

using namespace tb;

void setUp() {}
void tearDown() {}

// Synthetic curve: 1012 RPM/V, dead below cmd 40, 33 points at cmd = i*8 (cap 255).
static size_t makeCurve(CalPoint* out, int break_away, float slope) {
  size_t n = 0;
  for (int i = 0; i <= 32; i++) {
    int cmd = i * 8; if (cmd > 255) cmd = 255;
    float v = 3.3f * cmd / 255.0f;
    float rpm = (cmd >= break_away) ? slope * v : 0.0f;
    out[n++] = CalPoint{cmd, v, rpm};
  }
  return n;
}

void test_fit_recovers_known_slope() {
  CalPoint pts[33];
  size_t n = makeCurve(pts, 0, 1012.0f);   // no deadband -> pure line
  LinearFit f = fitLinear(pts, n);
  TEST_ASSERT_TRUE(f.valid);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 1012.0f, f.slope_rpm_per_volt);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, f.intercept_rpm);
}

void test_fit_invalid_on_too_few_points() {
  CalPoint pts[1] = {{0, 0.0f, 0.0f}};
  TEST_ASSERT_FALSE(fitLinear(pts, 1).valid);
  TEST_ASSERT_FALSE(fitLinear(pts, 0).valid);
}

void test_break_away_is_first_moving_command() {
  CalPoint pts[33];
  size_t n = makeCurve(pts, 40, 1012.0f);
  // cmd steps by 8, so the first point at/after 40 is 40 itself.
  TEST_ASSERT_EQUAL_INT(40, breakAwayCmd(pts, n, 5.0f));
}

void test_break_away_returns_minus_one_when_never_moves() {
  CalPoint pts[33];
  size_t n = makeCurve(pts, 9999, 1012.0f);  // never turns
  TEST_ASSERT_EQUAL_INT(-1, breakAwayCmd(pts, n, 5.0f));
}

void test_drop_out_is_last_moving_command_on_a_down_leg() {
  // Down-leg ordering: highest command first. Drop-out is where it stops.
  CalPoint pts[4] = {
    {100, 1.29f, 1300.0f},
    { 80, 1.03f,  900.0f},
    { 40, 0.52f,   30.0f},
    { 32, 0.41f,    0.0f},
  };
  TEST_ASSERT_EQUAL_INT(40, dropOutCmd(pts, 4, 5.0f));
}

void test_hysteresis_is_break_away_minus_drop_out() {
  // Stiction: it takes more command to start than to keep going.
  CalPoint up[3]   = {{40, 0.52f, 0.0f}, {48, 0.62f, 0.0f}, {56, 0.72f, 600.0f}};
  CalPoint down[3] = {{56, 0.72f, 600.0f}, {48, 0.62f, 480.0f}, {40, 0.52f, 0.0f}};
  int ba = breakAwayCmd(up, 3, 5.0f);
  int dr = dropOutCmd(down, 3, 5.0f);
  TEST_ASSERT_EQUAL_INT(56, ba);
  TEST_ASSERT_EQUAL_INT(48, dr);
  TEST_ASSERT_EQUAL_INT(8, ba - dr);
}

void test_knee_detects_saturation() {
  // Linear to cmd 160, then hard flat — the knee must land at the departure.
  CalPoint pts[5] = {
    { 40, 0.52f,  526.0f},
    { 80, 1.03f, 1042.0f},
    {120, 1.55f, 1569.0f},
    {160, 2.07f, 2095.0f},
    {200, 2.59f, 2100.0f},   // saturated: fit predicts ~2621
  };
  LinearFit f = fitLinear(pts, 4);       // fit only the linear span
  int knee = kneeCmd(pts, 5, f, 5.0f, 5.0f);
  TEST_ASSERT_EQUAL_INT(200, knee);
}

void test_knee_returns_minus_one_when_linear_throughout() {
  CalPoint pts[33];
  size_t n = makeCurve(pts, 0, 1012.0f);
  LinearFit f = fitLinear(pts, n);
  TEST_ASSERT_EQUAL_INT(-1, kneeCmd(pts, n, f, 5.0f, 5.0f));
}

void test_gear_ratio_exact() {
  // 10 wheel revolutions through a 15:1 box at 4096 cpr = 614400 motor counts.
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, gearRatio(614400, 10.0f, 4096));
}

void test_gear_ratio_detects_mislabelled_box() {
  // Same counts but only 8 wheel revs observed -> the box is not 15:1.
  float r = gearRatio(614400, 8.0f, 4096);
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 18.75f, r);
}

void test_gear_ratio_zero_revs_is_invalid() {
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, gearRatio(614400, 0.0f, 4096));
}

void test_gear_ratio_uses_magnitude() {
  // FR=LOW accumulates negative counts; the ratio is a magnitude.
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 15.0f, gearRatio(-614400, 10.0f, 4096));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_fit_recovers_known_slope);
  RUN_TEST(test_fit_invalid_on_too_few_points);
  RUN_TEST(test_break_away_is_first_moving_command);
  RUN_TEST(test_break_away_returns_minus_one_when_never_moves);
  RUN_TEST(test_drop_out_is_last_moving_command_on_a_down_leg);
  RUN_TEST(test_hysteresis_is_break_away_minus_drop_out);
  RUN_TEST(test_knee_detects_saturation);
  RUN_TEST(test_knee_returns_minus_one_when_linear_throughout);
  RUN_TEST(test_gear_ratio_exact);
  RUN_TEST(test_gear_ratio_detects_mislabelled_box);
  RUN_TEST(test_gear_ratio_zero_revs_is_invalid);
  RUN_TEST(test_gear_ratio_uses_magnitude);
  return UNITY_END();
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_motor_cal_analysis`
Expected: FAIL — compile error, `MotorCalAnalysis.h` not found.

- [ ] **Step 3: Write minimal implementation**

Create `lib/domain/MotorCalAnalysis.h`:

```cpp
// Arithmetic over a captured SV->RPM sweep. Deliberately free of Arduino and of
// the sweep state machine, so it can be tested on the host against synthetic
// curves whose answers are known in advance.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace tb {

struct CalPoint {
  int   cmd      = 0;      // 0..255 command
  float sv_volts = 0.0f;   // commanded SV
  float rpm      = 0.0f;   // measured motor-shaft RPM, sign as measured
};

struct LinearFit {
  float slope_rpm_per_volt = 0.0f;
  float intercept_rpm      = 0.0f;
  bool  valid              = false;
};

// Least-squares fit of |rpm| against sv_volts. valid=false if n < 2 or the
// commanded voltage never varies.
LinearFit fitLinear(const CalPoint* pts, size_t n);

// First point (in array order) whose |rpm| reaches rpm_floor. -1 if none.
// On an ascending leg this is break-away.
int breakAwayCmd(const CalPoint* pts, size_t n, float rpm_floor);

// Last point (in array order) whose |rpm| is still at/above rpm_floor. -1 if none.
// On a descending leg this is drop-out. Break-away minus drop-out is stiction.
int dropOutCmd(const CalPoint* pts, size_t n, float rpm_floor);

// First command where |rpm| departs the fit by more than tol_pct. -1 if the
// curve stays linear throughout. Points whose |rpm| is below rpm_floor are
// skipped — a stationary motor below break-away is not a saturation knee.
int kneeCmd(const CalPoint* pts, size_t n, const LinearFit& fit, float tol_pct, float rpm_floor);

// motor_counts / (cpr * wheel_revs). Uses |motor_counts| so FR=LOW works.
// Returns 0 when wheel_revs is 0 (caller treats as invalid).
float gearRatio(int32_t motor_counts, float wheel_revs, int32_t cpr);

} // namespace tb
```

Create `lib/domain/MotorCalAnalysis.cpp`:

```cpp
#include "MotorCalAnalysis.h"

namespace tb {

static inline float absf(float v) { return v < 0.0f ? -v : v; }

LinearFit fitLinear(const CalPoint* pts, size_t n) {
  LinearFit f;
  if (!pts || n < 2) return f;

  float sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (size_t i = 0; i < n; i++) {
    const float x = pts[i].sv_volts;
    const float y = absf(pts[i].rpm);
    sx += x; sy += y; sxx += x * x; sxy += x * y;
  }
  const float nn = (float)n;
  const float denom = nn * sxx - sx * sx;
  if (absf(denom) < 1e-9f) return f;   // no spread in x

  f.slope_rpm_per_volt = (nn * sxy - sx * sy) / denom;
  f.intercept_rpm      = (sy - f.slope_rpm_per_volt * sx) / nn;
  f.valid              = true;
  return f;
}

int breakAwayCmd(const CalPoint* pts, size_t n, float rpm_floor) {
  if (!pts) return -1;
  for (size_t i = 0; i < n; i++)
    if (absf(pts[i].rpm) >= rpm_floor) return pts[i].cmd;
  return -1;
}

int dropOutCmd(const CalPoint* pts, size_t n, float rpm_floor) {
  if (!pts) return -1;
  int last = -1;
  for (size_t i = 0; i < n; i++)
    if (absf(pts[i].rpm) >= rpm_floor) last = pts[i].cmd;
  return last;
}

int kneeCmd(const CalPoint* pts, size_t n, const LinearFit& fit, float tol_pct, float rpm_floor) {
  if (!pts || !fit.valid) return -1;
  for (size_t i = 0; i < n; i++) {
    const float measured = absf(pts[i].rpm);
    if (measured < rpm_floor) continue;           // stationary/deadband, not a knee
    const float predicted = fit.slope_rpm_per_volt * pts[i].sv_volts + fit.intercept_rpm;
    if (predicted <= 0.0f) continue;              // below the useful range
    const float err_pct = 100.0f * (predicted - measured) / predicted;
    if (err_pct > tol_pct) return pts[i].cmd;     // fell short of the line
  }
  return -1;
}

float gearRatio(int32_t motor_counts, float wheel_revs, int32_t cpr) {
  if (wheel_revs == 0.0f || cpr == 0) return 0.0f;
  const float counts = (float)(motor_counts < 0 ? -motor_counts : motor_counts);
  return counts / ((float)cpr * (wheel_revs < 0.0f ? -wheel_revs : wheel_revs));
}

} // namespace tb
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pio test -e native -f test_motor_cal_analysis`
Expected: PASS, 12 tests.

- [ ] **Step 5: Commit**

```bash
git add lib/domain/MotorCalAnalysis.h lib/domain/MotorCalAnalysis.cpp test/test_motor_cal_analysis/test_motor_cal_analysis.cpp
git commit -m "feat(bench): add MotorCalAnalysis — fit, break-away, hysteresis, knee, gear ratio"
```

---

### Task 3: Arm the stall trip in firmware

Wires Task 1 into the running rig, independent of any calibration routine. After this task the protection exists even for manual `s`/`+` driving.

**Files:**
- Modify: `src/bench_motor.cpp`

**Interfaces:**
- Consumes: `tb::StallDetector`, `tb::StallConfig` from Task 1
- Produces: globals `g_rpmFast` (float, RPM over a 100 ms window), `g_stall` (`tb::StallDetector`), `g_breakAwayCmd` (int, default 40), and `bool calGuardOk(const char* what)` which returns false and prints a reason when the brake is latched or the encoder is absent

**No `platformio.ini` change is needed.** `[env:bench_motor]` already sets
`lib_ignore = hal_esp32`, which excludes only the Arduino adapters; `lib/domain`
is pure C++17 and is picked up automatically by PlatformIO's LDF for both
`bench_motor` and `native`. Do **not** run `pio pkg install` to "fix" anything —
it rewrites `platformio.ini` and strips every comment from it.

- [ ] **Step 1: Add the fast RPM estimate and detector state**

In `src/bench_motor.cpp`, after the existing `#include <AS5600.h>` line add:

```cpp
#include "StallDetector.h"
#include "MotorCalAnalysis.h"
```

After the existing encoder globals (`g_rpm`), add:

```cpp
// Stall trip needs a faster RPM than the 500 ms status window: the trip fires at
// 250 ms, so a 500 ms estimate could not resolve it. 100 ms gives ~2.5 samples
// inside the trip window.
constexpr uint32_t RPM_FAST_MS = 100;
float    g_rpmFast   = 0.0f;
int32_t  g_fastPos   = 0;
uint32_t g_fastTime  = 0;

// Break-away defaults to 40 until calsweep measures the real value.
int g_breakAwayCmd = 40;
tb::StallDetector g_stall;
```

- [ ] **Step 2: Update the fast RPM and poll the detector in `loop()`**

Immediately after the existing encoder-sampling block in `loop()` (the `if (g_encOk && now - tEnc >= ENC_TICK_MS)` block), add:

```cpp
  if (g_encOk && now - g_fastTime >= RPM_FAST_MS) {
    const uint32_t dt = now - g_fastTime;
    if (g_fastTime && dt)
      g_rpmFast = (g_encPos - g_fastPos) * 60000.0f / ((float)ENC_CPR * dt);
    g_fastPos  = g_encPos;
    g_fastTime = now;

    // Armed whenever the driver is live. Not gated on any routine running —
    // manual `s`/`+` driving gets the same protection.
    if (g_enabled && !g_brake && g_stall.update(now, g_targetSv, g_rpmFast)) {
      Serial.printf("\n*** STALL TRIP at cmd %d — commanded but not turning ***\n",
                    g_stall.trippedAtCmd());
      Serial.println(F("*** SV cut, brake asserted. Check for a jam before retrying. ***"));
      estop();
    }
  }
```

- [ ] **Step 3: Open a grace window whenever the command rises**

In `setCommand()`, after `g_targetSv` is assigned, add:

```cpp
  if (g_targetSv > before) g_stall.noteCommandIncrease(millis());
```

and capture `before` at the top of the function:

```cpp
static void setCommand(int v, const char* why) {
  const int before = g_targetSv;
  g_targetSv = constrain(v, 0, CMD_MAX);
  if (g_targetSv > before) g_stall.noteCommandIncrease(millis());
  Serial.printf("> %s: cmd %d/255 (~%.2fV)\n",
                why, g_targetSv, ESP_VMAX * g_targetSv / (float)CMD_MAX);
}
```

- [ ] **Step 4: Reset the detector on enable, and add the shared guard**

In `cmdEnable`, replace the body with:

```cpp
static void cmdEnable(SerialCommands*) {
  g_enabled = true;
  g_stall.reset(millis());
  Serial.println(F("> ENABLE"));
}
```

Add near the other helpers, before the command handlers:

```cpp
// Every calibration routine refuses to start for the same two reasons, and both
// otherwise present as "it just sits there": `x` latches the brake with no
// auto-clear, and without an encoder there is no measurement and no stall trip.
static bool calGuardOk(const char* what) {
  if (!g_encOk) {
    Serial.printf("! %s needs the encoder — none detected (run `scan`)\n", what);
    return false;
  }
  if (g_brake) {
    Serial.printf("! %s refused: brake is latched. Send `n` to release, then retry.\n", what);
    return false;
  }
  return true;
}
```

- [ ] **Step 5: Build and flash**

Run: `pio run -e bench_motor -t upload`
Expected: SUCCESS. (`esp_idf_size: error: unrecognized arguments: --ng` is a cosmetic warning from the size reporter, not a build failure.)

- [ ] **Step 6: Verify the trip on hardware**

With the motor free to spin, run `pio device monitor -e bench_motor`, then `n`, `f`, `e`, `s 100`. Confirm normal running and no false trip during acceleration.
Then stop the shaft by hand (or leave the motor disconnected from the driver) and confirm within ~250 ms:

```
*** STALL TRIP at cmd 100 — commanded but not turning ***
```

Expected: SV collapses to 0, brake asserts, status shows `EN:off BRK:ON`.

- [ ] **Step 7: Commit**

```bash
git add src/bench_motor.cpp
git commit -m "feat(bench): arm StallDetector — replaces the rig's missing current limit"
```

---

### Task 4: `calsweep` — transfer curve

**Files:**
- Modify: `src/bench_motor.cpp`

**Interfaces:**
- Consumes: `calGuardOk()`, `g_stall`, `g_rpmFast` from Task 3; existing `g_encPos`, `g_encOk`, `g_enc`, `estop()`, `setCommand()`
- Produces: `enum class CalState`, globals `g_cal` (state), `g_calLeg`, `g_calIdx`, `g_calPrevRpm`, and `constexpr float GEAR_RATIO = 15.0f` (Task 6 uses it); console command `calsweep`

Note: the firmware does **not** use `tb::CalPoint` / `fitLinear` — it only emits
CSV. The Task 2 analysis runs on the host over that CSV, which is why it was worth
keeping out of the firmware in the first place.

- [ ] **Step 1: Add sweep state**

In `src/bench_motor.cpp`, after the stall globals from Task 3, add:

```cpp
// ── Calibration: transfer curve ──────────────────────────────────────────
// Four legs, each 33 points at cmd = min(i*8, 255): FR=HIGH up, HIGH down,
// LOW up, LOW down. The down-legs are what yield stiction hysteresis, and the
// two FR states independently verify that the encoder sign follows F/R.
constexpr uint32_t CAL_SETTLE_MS  = 800;
constexpr uint32_t CAL_MEASURE_MS = 400;
constexpr int      CAL_POINTS     = 33;
constexpr float    GEAR_RATIO     = 15.0f;

enum class CalState : uint8_t { Idle, Settle, Measure, Done };

CalState g_cal      = CalState::Idle;
int      g_calLeg   = 0;      // 0..3
int      g_calIdx   = 0;      // 0..32 within the leg
uint32_t g_calUntil = 0;
int32_t  g_calPos0  = 0;
uint32_t g_calT0    = 0;
float    g_calPrevRpm = 0.0f;   // previous point in this leg, for sag detection

static int calCmdForIndex(int i) { const int c = i * 8; return c > 255 ? 255 : c; }
static bool calLegIsHigh(int leg) { return leg < 2; }
static bool calLegIsUp(int leg)   { return (leg % 2) == 0; }

// Ascending legs walk 0..32; descending legs walk 32..0.
static int calCmdForLeg(int leg, int idx) {
  return calCmdForIndex(calLegIsUp(leg) ? idx : (CAL_POINTS - 1 - idx));
}
```

- [ ] **Step 2: Add the abort hook**

`estop()` must also cancel a running sweep, or `x` would stop the motor while the state machine kept stepping. In `estop()`, after `g_sweep = false;` add:

```cpp
  g_cal = CalState::Idle;
```

- [ ] **Step 3: Add the command handler**

```cpp
static void cmdCalSweep(SerialCommands*) {
  if (!calGuardOk("calsweep")) return;
  g_calLeg = 0; g_calIdx = 0; g_calPrevRpm = 0.0f;
  g_enabled = true;
  g_forward = true;                    // leg 0 is FR=HIGH
  g_stall.reset(millis());
  g_cal = CalState::Settle;
  g_calUntil = millis() + CAL_SETTLE_MS;
  setCommand(calCmdForLeg(0, 0), "calsweep");
  Serial.println(F("> CALSWEEP: 4 legs x 33 points, ~158s. Send x to abort."));
  Serial.println(F("CSV,sweep,fr_state,leg_dir,cmd,sv_volts,rpm_motor,rpm_wheel,counts_delta,agc,mag_status"));
}
```

Register it with the other multi-character commands:

```cpp
static SerialCommand c_calsweep("calsweep", cmdCalSweep);
```

and in `setup()` alongside the existing `AddCommand` calls:

```cpp
  g_cli.AddCommand(&c_calsweep);
```

- [ ] **Step 4: Drive the state machine from `loop()`**

Add before the status-print block in `loop()`:

```cpp
  // Non-blocking by construction: every dwell is a millis() comparison, never a
  // delay(). A blocking sweep would stop g_cli.ReadSerial() from running and
  // leave `x` dead for the full 158 s.
  if (g_cal == CalState::Settle && now >= g_calUntil) {
    g_calPos0 = g_encPos;
    g_calT0   = now;
    g_cal     = CalState::Measure;
    g_calUntil = now + CAL_MEASURE_MS;
  } else if (g_cal == CalState::Measure && now >= g_calUntil) {
    const uint32_t dt     = now - g_calT0;
    const int32_t  dcount = g_encPos - g_calPos0;
    const float    rpm    = dt ? dcount * 60000.0f / ((float)ENC_CPR * dt) : 0.0f;
    const int      cmd    = calCmdForLeg(g_calLeg, g_calIdx);
    const uint8_t  status = g_enc.readStatus();

    Serial.printf("CSV,sweep,%s,%s,%d,%.3f,%.1f,%.1f,%ld,%u,0x%02X\n",
                  calLegIsHigh(g_calLeg) ? "HIGH" : "LOW",
                  calLegIsUp(g_calLeg) ? "up" : "down",
                  cmd, ESP_VMAX * cmd / (float)CMD_MAX,
                  rpm, rpm / GEAR_RATIO, (long)dcount,
                  g_enc.readAGC(), status);

    // Supply-sag / driver current-limiting: on an ASCENDING leg, more command
    // must not produce less speed. When it does, the supply is sagging or the
    // driver is limiting internally — and the resulting bend looks exactly like
    // a genuine saturation knee in the fitted curve. Flag it at the point it
    // happens so the analysis is not silently reading a power problem as motor
    // physics. Threshold is 2% to clear ordinary point-to-point noise.
    if (calLegIsUp(g_calLeg) && g_calIdx > 0 &&
        fabsf(g_calPrevRpm) > 50.0f &&
        fabsf(rpm) < fabsf(g_calPrevRpm) * 0.98f) {
      Serial.printf("! SAG at cmd %d: %.1f -> %.1f RPM while command ROSE"
                    " (supply sagging or driver limiting — not a real knee)\n",
                    cmd, g_calPrevRpm, rpm);
    }
    g_calPrevRpm = rpm;

    // A weak magnet fails intermittently. Logging status per point turns a
    // silent mid-leg dropout into a visible abort instead of a plausible curve
    // with a hole in it.
    if (g_enc.magnetTooWeak() || !g_enc.magnetDetected()) {
      Serial.printf("! MAGNET FAULT at cmd %d — leg aborted, data unusable\n", cmd);
      estop();
    } else if (++g_calIdx >= CAL_POINTS) {
      g_calIdx = 0;
      if (++g_calLeg >= 4) {
        g_cal = CalState::Done;
        setCommand(0, "calsweep complete");
        g_enabled = false;
        Serial.println(F("> CALSWEEP complete."));
      } else {
        g_forward = calLegIsHigh(g_calLeg);
        setCommand(calCmdForLeg(g_calLeg, 0), "calsweep leg");
        g_stall.reset(now);          // direction change: restart trip timing
        g_calPrevRpm = 0.0f;         // sag comparison must not cross legs
        g_cal = CalState::Settle;
        g_calUntil = now + CAL_SETTLE_MS;
      }
    } else {
      setCommand(calCmdForLeg(g_calLeg, g_calIdx), "calsweep");
      g_cal = CalState::Settle;
      g_calUntil = now + CAL_SETTLE_MS;
    }
  }
```

- [ ] **Step 5: Build and flash**

Run: `pio run -e bench_motor -t upload`
Expected: SUCCESS.

- [ ] **Step 6: Verify on hardware**

Run `pio device monitor -e bench_motor`, then `n`, `calsweep`.
Expected: the CSV header, then rows appearing roughly every 1.2 s. Confirm `x` aborts mid-run and the motor stops immediately.
Sanity check against 2026-07-27 measurements — the `FR=HIGH,up` rows near cmd 60 / 100 / 160 should read approximately 789 / 1305 / 2067 motor RPM.

- [ ] **Step 7: Commit**

```bash
git add src/bench_motor.cpp
git commit -m "feat(bench): add calsweep — 4-leg SV/RPM transfer curve with per-point magnet health"
```

---

### Task 5: `calstep` — step response

**Files:**
- Modify: `src/bench_motor.cpp`

**Interfaces:**
- Consumes: `calGuardOk()`, `g_stall`, `g_encPos` from Task 3
- Produces: `enum class StepPhase`, globals `g_step`, `g_stepCmd`; console command `calstep <cmd>`

- [ ] **Step 1: Add step-capture state**

```cpp
// ── Calibration: step response ───────────────────────────────────────────
// Logs at the encoder sample rate, not the 2 Hz status rate — a time constant
// cannot be extracted from 2 samples per second.
constexpr uint32_t STEP_LOG_MS   = 4;
constexpr uint32_t STEP_RISE_MS  = 2000;
constexpr uint32_t STEP_STOP_MS  = 6000;   // cap on coast/brake captures
constexpr float    STEP_STOP_RPM = 5.0f;   // "stopped" threshold

enum class StepPhase : uint8_t { Idle, Rise, Settle, Coast, Brake, Done };

StepPhase g_step     = StepPhase::Idle;
int       g_stepCmd  = 0;
uint32_t  g_stepT0   = 0;
uint32_t  g_stepNext = 0;
```

- [ ] **Step 2: Add the command handler**

```cpp
static void cmdCalStep(SerialCommands* s) {
  if (!calGuardOk("calstep")) return;
  const char* arg = s->Next();
  if (!arg) { Serial.println(F("? usage: calstep <0-255>")); return; }
  g_stepCmd = constrain(atoi(arg), 0, CMD_MAX);
  if (g_stepCmd < g_breakAwayCmd) {
    Serial.printf("! cmd %d is below break-away (%d) — it will not turn\n",
                  g_stepCmd, g_breakAwayCmd);
    return;
  }
  g_enabled = true; g_forward = true; g_brake = false;
  g_stall.reset(millis());
  g_step   = StepPhase::Rise;
  g_stepT0 = millis();
  g_stepNext = g_stepT0;
  setCommand(g_stepCmd, "calstep rise");
  Serial.println(F("> CALSTEP: rise -> coast -> brake. Send x to abort."));
  Serial.println(F("CSV,step,phase,t_ms,cmd,rpm_motor"));
}

static SerialCommand c_calstep("calstep", cmdCalStep);
```

Register in `setup()`:

```cpp
  g_cli.AddCommand(&c_calstep);
```

- [ ] **Step 3: Cancel on e-stop**

In `estop()`, alongside the `g_cal` line from Task 4, add:

```cpp
  g_step = StepPhase::Idle;
```

- [ ] **Step 4: Drive the captures from `loop()`**

```cpp
  if (g_step != StepPhase::Idle && g_step != StepPhase::Done && now >= g_stepNext) {
    g_stepNext = now + STEP_LOG_MS;
    const char* phase = (g_step == StepPhase::Rise)  ? "rise"
                      : (g_step == StepPhase::Coast) ? "coast"
                      : (g_step == StepPhase::Brake) ? "brake" : "settle";
    if (g_step != StepPhase::Settle)
      Serial.printf("CSV,step,%s,%lu,%d,%.1f\n",
                    phase, (unsigned long)(now - g_stepT0), g_targetSv, g_rpmFast);

    const bool stopped = fabsf(g_rpmFast) < STEP_STOP_RPM;
    switch (g_step) {
      case StepPhase::Rise:
        if (now - g_stepT0 >= STEP_RISE_MS) {
          g_step = StepPhase::Settle; g_stepT0 = now;   // hold before coasting
        }
        break;
      case StepPhase::Settle:
        if (now - g_stepT0 >= 500) {
          // Coast = disabled with the brake OFF: pure mechanical friction.
          g_enabled = false; g_brake = false;
          g_step = StepPhase::Coast; g_stepT0 = now;
        }
        break;
      case StepPhase::Coast:
        if (stopped || now - g_stepT0 >= STEP_STOP_MS) {
          // Spin back up, then brake, so coast and brake are compared from the
          // same starting speed.
          g_enabled = true; g_brake = false;
          setCommand(g_stepCmd, "calstep respin");
          g_stall.reset(now);
          g_step = StepPhase::Brake; g_stepT0 = now;
        }
        break;
      case StepPhase::Brake:
        // Give it STEP_RISE_MS to reach speed again, then slam the brake.
        if (now - g_stepT0 >= STEP_RISE_MS && !g_brake) g_brake = true;
        if (g_brake && (stopped || now - g_stepT0 >= STEP_RISE_MS + STEP_STOP_MS)) {
          setCommand(0, "calstep complete");
          g_enabled = false;
          g_step = StepPhase::Done;
          Serial.println(F("> CALSTEP complete."));
        }
        break;
      default: break;
    }
  }
```

- [ ] **Step 5: Build and flash**

Run: `pio run -e bench_motor -t upload`
Expected: SUCCESS.

- [ ] **Step 6: Verify on hardware**

Run `calstep 100`. Expected: dense `CSV,step,rise,...` rows at ~4 ms spacing rising to ~1305 RPM, then `coast` rows decaying slowly, then `brake` rows decaying sharply. Brake decay must be visibly faster than coast — that difference is the brake's contribution.
Confirm `x` aborts mid-capture.

- [ ] **Step 7: Commit**

```bash
git add src/bench_motor.cpp
git commit -m "feat(bench): add calstep — rise/coast/brake response logged at 250Hz"
```

---

### Task 6: `calgear` — gearbox verification

**Files:**
- Modify: `src/bench_motor.cpp`

**Interfaces:**
- Consumes: `calGuardOk()`, `tb::gearRatio()` from Task 2, `GEAR_RATIO` from Task 4
- Produces: console commands `calgear [cmd]` and `revs <n>`

- [ ] **Step 1: Add gear-run state**

```cpp
// ── Calibration: gearbox verification ────────────────────────────────────
// Deliberately NOT "stop at exactly 61440 counts": the shaft coasts after power
// is cut, so the resting position would not equal the target and the coast would
// be misread as gearbox error. Instead run freely, keep counting THROUGH the
// coast (the encoder does), and divide by the revolutions actually observed.
constexpr float GEAR_TARGET_REVS = 10.0f;

enum class GearPhase : uint8_t { Idle, Running, Stopping, AwaitCount };

GearPhase g_gear      = GearPhase::Idle;
uint32_t  g_gearUntil = 0;
int32_t   g_gearPos0  = 0;
int32_t   g_gearCounts = 0;
```

- [ ] **Step 2: Add the handlers**

```cpp
static void cmdCalGear(SerialCommands* s) {
  if (!calGuardOk("calgear")) return;
  const char* arg = s->Next();
  const int cmd = arg ? constrain(atoi(arg), 0, CMD_MAX) : 60;
  if (cmd < g_breakAwayCmd) {
    Serial.printf("! cmd %d is below break-away (%d) — it will not turn\n",
                  cmd, g_breakAwayCmd);
    return;
  }
  // Duration for ~10 predicted wheel revs, from the measured 1012 RPM/V curve.
  const float sv         = ESP_VMAX * cmd / (float)CMD_MAX;
  const float motor_rpm  = 1012.0f * sv;
  const float wheel_rps  = motor_rpm / GEAR_RATIO / 60.0f;
  const uint32_t run_ms  = (wheel_rps > 0.01f)
                         ? (uint32_t)(GEAR_TARGET_REVS / wheel_rps * 1000.0f) : 12000;

  g_enabled = true; g_forward = true; g_brake = false;
  g_enc.resetCumulativePosition(0);
  g_encPos = g_gearPos0 = 0;
  g_stall.reset(millis());
  setCommand(cmd, "calgear");
  g_gear = GearPhase::Running;
  g_gearUntil = millis() + run_ms;
  Serial.printf("> CALGEAR: cmd %d for %lums (~%.0f predicted wheel revs).\n",
                cmd, (unsigned long)run_ms, GEAR_TARGET_REVS);
  Serial.println(F("  MARK THE WHEEL and count its revolutions. Send x to abort."));
}

static void cmdRevs(SerialCommands* s) {
  if (g_gear != GearPhase::AwaitCount) {
    Serial.println(F("? `revs` only applies right after a calgear run"));
    return;
  }
  const char* arg = s->Next();
  if (!arg) { Serial.println(F("? usage: revs <wheel revolutions observed>")); return; }
  const float revs = atof(arg);
  const float measured = tb::gearRatio(g_gearCounts, revs, ENC_CPR);
  if (measured <= 0.0f) { Serial.println(F("? revolutions must be > 0")); return; }
  const float err = 100.0f * (measured - GEAR_RATIO) / GEAR_RATIO;
  Serial.println(F("CSV,gear,motor_counts,wheel_revs_reported,measured_ratio,label_ratio,error_pct"));
  Serial.printf("CSV,gear,%ld,%.2f,%.4f,%.2f,%+.2f\n",
                (long)g_gearCounts, revs, measured, GEAR_RATIO, err);
  if (fabsf(err) > 5.0f)
    Serial.println(F("! >5% from the 15:1 label — suspect a mislabelled or wrong-fitted box"));
  g_gear = GearPhase::Idle;
}

static SerialCommand c_calgear("calgear", cmdCalGear);
static SerialCommand c_revs("revs", cmdRevs);
```

Register in `setup()`:

```cpp
  g_cli.AddCommand(&c_calgear);  g_cli.AddCommand(&c_revs);
```

- [ ] **Step 3: Cancel on e-stop**

In `estop()`, alongside the `g_cal` and `g_step` lines:

```cpp
  g_gear = GearPhase::Idle;
```

- [ ] **Step 4: Drive it from `loop()`**

```cpp
  if (g_gear == GearPhase::Running && now >= g_gearUntil) {
    setCommand(0, "calgear stopping");
    g_enabled = false; g_brake = true;
    g_gear = GearPhase::Stopping;
    g_gearUntil = now + 3000;          // let it come fully to rest
  } else if (g_gear == GearPhase::Stopping && now >= g_gearUntil) {
    // Counts accumulated through the coast are INCLUDED — that is the point.
    g_gearCounts = g_encPos - g_gearPos0;
    g_brake = false;
    g_gear = GearPhase::AwaitCount;
    Serial.printf("> CALGEAR done: %ld motor counts (%.2f motor revs).\n",
                  (long)g_gearCounts, (float)g_gearCounts / ENC_CPR);
    Serial.println(F("  Now send:  revs <wheel revolutions you counted>"));
  }
```

- [ ] **Step 5: Update the help text**

Replace the `with Enter:` block inside `printHelp()` with:

```cpp
    " with Enter:\n"
    "   s <0-255>  set command      v <volts>  set by SV volts\n"
    "   sweep      auto-ramp 0 -> max -> 0 (any key cancels)\n"
    "   scan       re-run the I2C bus scan\n"
    "   calsweep   transfer curve, 4 legs, ~158s\n"
    "   calstep <cmd>  rise/coast/brake response at 250Hz\n"
    "   calgear [cmd]  gearbox check (then: revs <n>)\n"
```

- [ ] **Step 6: Build and flash**

Run: `pio run -e bench_motor -t upload`
Expected: SUCCESS.

- [ ] **Step 7: Verify on hardware**

Mark the wheel. Run `calgear`. Count wheel revolutions during the run, then send e.g. `revs 10`.
Expected: a `CSV,gear,...` row with `measured_ratio` near 15.0 and `error_pct` within ±5%.

- [ ] **Step 8: Commit**

```bash
git add src/bench_motor.cpp
git commit -m "feat(bench): add calgear — 15:1 gearbox verification via counted wheel revs"
```

---

## After implementation

The magnet airgap must be corrected before any of this data is kept: AGC is currently railed at 128/128 (3.3 V full scale), meaning the magnet sits too far from the chip. `calsweep` will abort a leg on a magnet fault rather than record a hole, but a railed AGC has no margin to begin with. Target AGC ~64, verified with `m`.
