// Host tests for CornerEdgeDetector — band threshold + debounce on a solid board.
#include <unity.h>

#include "CornerEdgeDetector.h"

using namespace fal;

namespace {
AlignmentFrame frame4(uint16_t fl, uint16_t fr, uint16_t rl, uint16_t rr, bool valid = true) {
  AlignmentFrame f;
  f.zone_count = 4;
  f.zones[0] = {fl, valid};
  f.zones[1] = {fr, valid};
  f.zones[2] = {rl, valid};
  f.zones[3] = {rr, valid};
  return f;
}
} // namespace

void setUp() {}
void tearDown() {}

// A reading in the band flips "present" only after `debounce` agreeing samples.
void test_present_after_debounce() {
  CornerConfig cfg;  // band 20..400, debounce 2
  CornerEdgeDetector d(cfg);

  d.update(frame4(100, 100, 100, 100));
  TEST_ASSERT_FALSE(d.present(Corner::FL));  // 1 sample < debounce
  d.update(frame4(100, 100, 100, 100));
  TEST_ASSERT_TRUE(d.present(Corner::FL));   // 2 samples -> flip
  TEST_ASSERT_TRUE(d.present(Corner::RR));
}

// A single out-of-band spike does not flip an established "present".
void test_noise_spike_ignored() {
  CornerConfig cfg;
  CornerEdgeDetector d(cfg);
  d.update(frame4(100, 100, 100, 100));
  d.update(frame4(100, 100, 100, 100));
  TEST_ASSERT_TRUE(d.present(Corner::FL));

  d.update(frame4(2000, 100, 100, 100));  // one spike on FL (out of band)
  TEST_ASSERT_TRUE(d.present(Corner::FL));  // debounce absorbs it
  d.update(frame4(2000, 100, 100, 100));  // second -> now flips off
  TEST_ASSERT_FALSE(d.present(Corner::FL));
}

// Out-of-band distance and invalid readings are treated as "no board".
void test_out_of_band_and_invalid() {
  CornerConfig cfg;
  CornerEdgeDetector d(cfg);
  d.update(frame4(5, 500, 100, 100));   // FL below min, FR above max
  d.update(frame4(5, 500, 100, 100));
  TEST_ASSERT_FALSE(d.present(Corner::FL));
  TEST_ASSERT_FALSE(d.present(Corner::FR));
  TEST_ASSERT_TRUE(d.present(Corner::RL));

  CornerEdgeDetector d2(cfg);
  d2.update(frame4(100, 100, 100, 100, /*valid=*/false));
  d2.update(frame4(100, 100, 100, 100, /*valid=*/false));
  TEST_ASSERT_FALSE(d2.present(Corner::FL));  // invalid never counts as present
}


// --- hysteresis: a corner near a band edge must not chatter -----------------

// 250 mm is inside the release band but outside the assert band, so it cannot
// turn a corner ON. This is the defence against a distant object (arm, ceiling,
// wall) being mistaken for the trolley.
void test_hysteresis_assert_band_rejects_far_reading() {
  CornerConfig cfg;
  cfg.assert_min_mm = 45;  cfg.assert_max_mm = 200;
  cfg.band_min_mm = 30;    cfg.band_max_mm = 300;
  cfg.debounce = 2;
  CornerEdgeDetector d(cfg);

  for (int i = 0; i < 10; ++i) d.update(frame4(250, 250, 250, 250));
  TEST_ASSERT_FALSE(d.present(Corner::FL));
}

// ...but the SAME 250 mm keeps an established corner on, because release uses
// the wider band. Crossing in and crossing out happen at different distances.
void test_hysteresis_holds_established_corner() {
  CornerConfig cfg;
  cfg.assert_min_mm = 45;  cfg.assert_max_mm = 200;
  cfg.band_min_mm = 30;    cfg.band_max_mm = 300;
  cfg.debounce = 2;
  CornerEdgeDetector d(cfg);

  d.update(frame4(100, 100, 100, 100));
  d.update(frame4(100, 100, 100, 100));
  TEST_ASSERT_TRUE(d.present(Corner::FL));

  for (int i = 0; i < 10; ++i) d.update(frame4(250, 250, 250, 250));
  TEST_ASSERT_TRUE(d.present(Corner::FL));   // held: inside release, outside assert

  for (int i = 0; i < 10; ++i) d.update(frame4(350, 350, 350, 350));
  TEST_ASSERT_FALSE(d.present(Corner::FL));  // outside release -> drops
}

// Asymmetric debounce: releasing may demand more evidence than asserting.
void test_release_debounce_independent() {
  CornerConfig cfg;
  cfg.debounce = 2;
  cfg.release_debounce = 5;
  CornerEdgeDetector d(cfg);

  d.update(frame4(100, 100, 100, 100));
  d.update(frame4(100, 100, 100, 100));
  TEST_ASSERT_TRUE(d.present(Corner::FL));

  for (int i = 0; i < 4; ++i) d.update(frame4(8190, 8190, 8190, 8190));
  TEST_ASSERT_TRUE(d.present(Corner::FL));   // 4 < release_debounce
  d.update(frame4(8190, 8190, 8190, 8190));
  TEST_ASSERT_FALSE(d.present(Corner::FL));  // 5th drops it
}

// Zeroed hysteresis/release fields must behave exactly as before they existed.
void test_defaults_unchanged_by_new_fields() {
  CornerConfig cfg;  // assert_* = 0, release_debounce = 0
  CornerEdgeDetector d(cfg);
  d.update(frame4(390, 390, 390, 390));
  d.update(frame4(390, 390, 390, 390));
  TEST_ASSERT_TRUE(d.present(Corner::FL));   // 390 is inside the plain 20..400 band
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_present_after_debounce);
  RUN_TEST(test_noise_spike_ignored);
  RUN_TEST(test_out_of_band_and_invalid);
  RUN_TEST(test_hysteresis_assert_band_rejects_far_reading);
  RUN_TEST(test_hysteresis_holds_established_corner);
  RUN_TEST(test_release_debounce_independent);
  RUN_TEST(test_defaults_unchanged_by_new_fields);
  return UNITY_END();
}
