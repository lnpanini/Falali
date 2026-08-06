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

void test_does_not_trip_before_trip_ms() {
  // The other timing tests only assert a trip fires SOMEWHERE across a long
  // sweep, so none of them would catch a trip that fires early. Pin the exact
  // instant instead: the first update() call at a stall condition only STARTS
  // the window and must return false, so the earliest legal trip is exactly
  // trip_ms after that starting call, not a tick sooner.
  StallDetector d(cfg());
  d.reset(0);
  for (uint32_t t = 0; t < 250; ++t) {
    TEST_ASSERT_FALSE(d.update(t, 100, 0.0f));
  }
  TEST_ASSERT_FALSE(d.tripped());
  TEST_ASSERT_TRUE(d.update(250, 100, 0.0f));
  TEST_ASSERT_TRUE(d.tripped());
  TEST_ASSERT_EQUAL_INT(100, d.trippedAtCmd());
}

void test_break_away_boundary_is_inclusive() {
  // cmd == break_away_cmd must still be policed: the implementation only
  // skips policing when cmd < break_away_cmd (StallDetector.cpp:24), so the
  // boundary value itself is NOT exempt. A sustained stall at exactly the
  // break-away command must trip.
  StallDetector d(cfg());
  d.reset(0);
  bool edge = false;
  for (uint32_t t = 0; t <= 300; t += 10) {
    if (d.update(t, 40, 0.0f)) edge = true;
  }
  TEST_ASSERT_TRUE(edge);
  TEST_ASSERT_TRUE(d.tripped());
  TEST_ASSERT_EQUAL_INT(40, d.trippedAtCmd());
}

void test_survives_millis_wraparound() {
  // millis() rolls over to 0 roughly every 49 days. The code relies on a
  // signed-cast idiom ("(int32_t)(now_ms - grace_until_) < 0", StallDetector.cpp:27)
  // to keep "now vs. deadline" comparisons correct across that rollover. A
  // naive `now_ms < grace_until_` replacement breaks the instant grace_until_
  // has already wrapped to a small raw value while now_ms has not wrapped yet.
  // This test starts right at the rollover boundary and checks both the stall
  // timer and the grace window survive crossing it.

  // --- stall timing survives the wrap ---
  StallDetector d(cfg());
  const uint32_t t0 = 0xFFFFFF00;     // 256ms before millis() rolls over to 0
  d.reset(t0);
  const uint32_t start = t0 + 0x20;   // 32ms before rollover; first stalled update
  bool edge = false;
  for (uint32_t off = 0; off <= 300; off += 25) {
    uint32_t t = start + off;         // crosses the UINT32_MAX -> 0 rollover partway through
    if (d.update(t, 100, 0.0f)) edge = true;
  }
  TEST_ASSERT_TRUE(edge);
  TEST_ASSERT_TRUE(d.tripped());
  TEST_ASSERT_EQUAL_INT(100, d.trippedAtCmd());

  // --- grace suppression survives the wrap ---
  StallDetector d2(cfg());
  d2.reset(t0);
  d2.noteCommandIncrease(t0);  // grace_until_ = t0 + 300, which itself wraps to a small value
  for (uint32_t off = 0; off < 300; off += 25) {
    uint32_t t = t0 + off;     // still inside the 300ms grace window throughout, wrap included
    TEST_ASSERT_FALSE(d2.update(t, 100, 0.0f));
  }
  TEST_ASSERT_FALSE(d2.tripped());
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
  RUN_TEST(test_does_not_trip_before_trip_ms);
  RUN_TEST(test_break_away_boundary_is_inclusive);
  RUN_TEST(test_survives_millis_wraparound);
  return UNITY_END();
}
