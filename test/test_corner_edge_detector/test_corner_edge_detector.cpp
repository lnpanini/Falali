// Host tests for CornerEdgeDetector — band threshold + debounce on a solid board.
#include <unity.h>

#include "CornerEdgeDetector.h"

using namespace tb;

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

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_present_after_debounce);
  RUN_TEST(test_noise_spike_ignored);
  RUN_TEST(test_out_of_band_and_invalid);
  return UNITY_END();
}
