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
  int knee = kneeCmd(pts, 5, f, 5.0f);
  TEST_ASSERT_EQUAL_INT(200, knee);
}

void test_knee_returns_minus_one_when_linear_throughout() {
  CalPoint pts[33];
  size_t n = makeCurve(pts, 0, 1012.0f);
  LinearFit f = fitLinear(pts, n);
  TEST_ASSERT_EQUAL_INT(-1, kneeCmd(pts, n, f, 5.0f));
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
