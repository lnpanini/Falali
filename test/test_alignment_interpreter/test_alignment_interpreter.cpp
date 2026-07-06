// Host tests for AlignmentInterpreter — the confidence logic that must survive an
// intermittent (caged/barred) trolley underside.
#include <unity.h>

#include "AlignmentInterpreter.h"
#include "FakeClock.h"

using namespace tb;

namespace {

AlignmentFrame frame1(uint16_t mm, bool valid, uint32_t t) {
  AlignmentFrame f;
  f.zone_count = 1;
  f.t_ms = t;
  f.zones[0] = {mm, valid};
  return f;
}

AlignmentFrame frame2(uint16_t lmm, bool lv, uint16_t rmm, bool rv, uint32_t t) {
  AlignmentFrame f;
  f.zone_count = 2;
  f.t_ms = t;
  f.zones[0] = {lmm, lv};
  f.zones[1] = {rmm, rv};
  return f;
}

} // namespace

void setUp() {}
void tearDown() {}

// Intermittent dropouts (a few gaps between bars) must NOT collapse under_trolley.
void test_dropouts_keep_under_trolley_high() {
  FakeClock clk;
  AlignmentConfig cfg;  // band 20..150, window 16, threshold 0.6
  AlignmentInterpreter in(cfg, clk);

  uint32_t t = 0;
  for (int i = 0; i < 16; ++i) {
    clk.set(t);
    const bool dropout = (i == 3 || i == 7 || i == 11);  // 3 of 16 invalid
    in.update(frame1(80, !dropout, t));
    t += 20;
  }

  const AlignmentState s = in.state();
  TEST_ASSERT_TRUE(s.under_trolley > 0.75f);  // 13/16 = 0.81
  TEST_ASSERT_TRUE(s.fresh);
}

// Sustained invalid returns for longer than the freshness timeout clear `fresh`.
void test_sustained_invalids_clear_fresh() {
  FakeClock clk;
  AlignmentConfig cfg;  // freshness_timeout_ms = 300
  AlignmentInterpreter in(cfg, clk);

  clk.set(0);
  in.update(frame1(80, true, 0));
  TEST_ASSERT_TRUE(in.state().fresh);

  clk.set(400);  // 400ms since last valid > 300ms timeout
  in.update(frame1(0, false, 400));
  TEST_ASSERT_FALSE(in.state().fresh);
}

// clamp_safe only asserts after the debounce time of continuously-safe conditions.
void test_clamp_safe_requires_debounce() {
  FakeClock clk;
  AlignmentConfig cfg;
  cfg.zone_side[0] = -1;  // left
  cfg.zone_side[1] = +1;  // right
  cfg.clamp_debounce_ms = 400;
  AlignmentInterpreter in(cfg, clk);

  clk.set(0);
  in.update(frame2(80, true, 80, true, 0));  // symmetric, in-band
  TEST_ASSERT_FALSE(in.state().clamp_safe);  // 0ms < 400ms

  clk.set(399);
  in.update(frame2(80, true, 80, true, 399));
  TEST_ASSERT_FALSE(in.state().clamp_safe);

  clk.set(400);
  in.update(frame2(80, true, 80, true, 400));
  const AlignmentState s = in.state();
  TEST_ASSERT_TRUE(s.clamp_safe);
  TEST_ASSERT_TRUE(s.under_trolley > 0.9f);
  TEST_ASSERT_TRUE(s.centred > 0.9f);
}

// Off-centre coverage yields a signed lateral bias and denies clamping.
void test_off_centre_denies_clamp() {
  FakeClock clk;
  AlignmentConfig cfg;
  cfg.zone_side[0] = -1;  // left
  cfg.zone_side[1] = +1;  // right
  AlignmentInterpreter in(cfg, clk);

  uint32_t t = 0;
  for (int i = 0; i < 16; ++i) {  // right sees underside, left sees gaps
    clk.set(t);
    in.update(frame2(0, false, 80, true, t));
    t += 50;
  }

  const AlignmentState s = in.state();
  TEST_ASSERT_TRUE(s.lateral > 0.4f);   // right-heavy -> positive
  TEST_ASSERT_TRUE(s.centred < 0.6f);
  TEST_ASSERT_FALSE(s.clamp_safe);
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_dropouts_keep_under_trolley_high);
  RUN_TEST(test_sustained_invalids_clear_fresh);
  RUN_TEST(test_clamp_safe_requires_debounce);
  RUN_TEST(test_off_centre_denies_clamp);
  return UNITY_END();
}
