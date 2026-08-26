// Host tests for the motor calibration arithmetic, fed synthetic curves with
// known answers. The bench measured 1012 RPM/V on 2026-07-27, so that slope is
// used as the realistic case throughout.
#include <unity.h>

#include "MotorCalAnalysis.h"

using namespace fal;

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

// Same full sweep shape (33 points, dead below break_away) but the RPM flattens
// to a constant ceiling above knee_cmd, so there is a genuine saturation knee
// after a genuine deadband.
static size_t makeSaturatingCurve(CalPoint* out, int break_away, float slope, int knee_cmd) {
  size_t n = 0;
  float ceiling = 0.0f;
  for (int i = 0; i <= 32; i++) {
    int cmd = i * 8; if (cmd > 255) cmd = 255;
    float v = 3.3f * cmd / 255.0f;
    float rpm;
    if (cmd < break_away) {
      rpm = 0.0f;
    } else if (cmd <= knee_cmd) {
      rpm = slope * v;
      ceiling = rpm;
    } else {
      rpm = ceiling;
    }
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
  //
  // tol_pct and rpm_floor are deliberately DISTINCT here (5.0 / 20.0), not the
  // 5.0/5.0 every call site used to share. With identical values, swapping the
  // two parameters at the call site is invisible: nothing in the test would
  // fail regardless of which one lands in which slot. The inserted cmd=90
  // sample (10 RPM — a below-floor dropout sitting between the fit's later
  // points and the saturation point) is what makes that swap visible: with the
  // arguments in the RIGHT order, rpm_floor=20 skips it (10 < 20, ordinary
  // deadband noise) and the knee is still correctly found at cmd 200. Swap the
  // two literals below (kneeCmd(pts, 6, f, 20.0f, 5.0f)) and rpm_floor becomes
  // 5: the noise sample is no longer skipped (10 >= 5), its ~99% deviation
  // trips against the now-20% tolerance, and the function returns 90 instead
  // of 200 — the assertion below fails. Manually confirmed by temporarily
  // swapping the two arguments and re-running `pio test -e native`.
  CalPoint pts[6] = {
    { 40, 0.52f,  526.0f},
    { 80, 1.03f, 1042.0f},
    {120, 1.55f, 1569.0f},
    {160, 2.07f, 2095.0f},
    { 90, 1.165f,  10.0f},   // below-floor noise dropout, NOT part of the fit
    {200, 2.59f, 2100.0f},   // saturated: fit predicts ~2621
  };
  LinearFit f = fitLinear(pts, 4);       // fit only the linear span (unaffected by the noise point)
  int knee = kneeCmd(pts, 6, f, 5.0f, 20.0f);
  TEST_ASSERT_EQUAL_INT(200, knee);
}

void test_knee_returns_minus_one_when_linear_throughout() {
  CalPoint pts[33];
  size_t n = makeCurve(pts, 0, 1012.0f);
  LinearFit f = fitLinear(pts, n);
  TEST_ASSERT_EQUAL_INT(-1, kneeCmd(pts, n, f, 5.0f, 5.0f));
}

void test_knee_ignores_deadband_before_break_away() {
  // Regression test: a motor that is dead below break-away and perfectly
  // linear above it must NOT read as saturated at the first commanded point.
  CalPoint pts[33];
  size_t n = makeCurve(pts, 40, 1012.0f);
  // Fit only the moving points (cmd >= 40, indices 5..32) — the deadband
  // itself is never part of a real fit.
  LinearFit f = fitLinear(pts + 5, n - 5);
  TEST_ASSERT_TRUE(f.valid);
  // Call kneeCmd on the FULL sweep, deadband included. The curve is linear
  // throughout its operating region, so there is no knee.
  TEST_ASSERT_EQUAL_INT(-1, kneeCmd(pts, n, f, 5.0f, 5.0f));
}

void test_knee_finds_saturation_past_a_deadband() {
  // Same full sweep shape, dead below cmd 40, but flattened to a ceiling
  // above cmd 80 — a genuine knee after a genuine deadband.
  CalPoint pts[33];
  size_t n = makeSaturatingCurve(pts, 40, 1012.0f, 80);
  // Fit only the moving, still-linear points: cmd 40..80 (indices 5..10).
  LinearFit f = fitLinear(pts + 5, 6);
  TEST_ASSERT_TRUE(f.valid);
  int knee = kneeCmd(pts, n, f, 5.0f, 5.0f);
  TEST_ASSERT_EQUAL_INT(88, knee);   // first saturated command, not a deadband one
}

void test_fit_invalid_when_voltage_never_varies() {
  // n >= 2, but sv_volts is identical on every point -> no spread in x.
  CalPoint pts[3] = {
    {100, 1.5f, 500.0f},
    {150, 1.5f, 700.0f},
    {200, 1.5f, 900.0f},
  };
  TEST_ASSERT_FALSE(fitLinear(pts, 3).valid);
}

void test_gear_ratio_zero_cpr_is_invalid() {
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.0f, gearRatio(614400, 10.0f, 0));
}

void test_knee_returns_minus_one_on_invalid_fit() {
  CalPoint pts[3] = {
    { 40, 0.52f,  526.0f},
    { 80, 1.03f, 1042.0f},
    {120, 1.55f, 1569.0f},
  };
  LinearFit invalid_fit{};   // default-constructed: valid == false
  TEST_ASSERT_EQUAL_INT(-1, kneeCmd(pts, 3, invalid_fit, 5.0f, 5.0f));
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
  RUN_TEST(test_knee_ignores_deadband_before_break_away);
  RUN_TEST(test_knee_finds_saturation_past_a_deadband);
  RUN_TEST(test_fit_invalid_when_voltage_never_varies);
  RUN_TEST(test_gear_ratio_zero_cpr_is_invalid);
  RUN_TEST(test_knee_returns_minus_one_on_invalid_fit);
  RUN_TEST(test_gear_ratio_exact);
  RUN_TEST(test_gear_ratio_detects_mislabelled_box);
  RUN_TEST(test_gear_ratio_zero_revs_is_invalid);
  RUN_TEST(test_gear_ratio_uses_magnitude);
  return UNITY_END();
}
