// Host tests for LinkWatchdog — the dead-man's handle on the Pi 5 control link.
//
// The properties under test are the ones that decide whether a robot with a
// silent brain keeps driving. Each test names the real-world failure it prevents.
#include <unity.h>

#include "LinkWatchdog.h"

using namespace tb;

void setUp() {}
void tearDown() {}

static LinkConfig cfg(uint32_t timeout_ms = 100) {
  LinkConfig c;
  c.timeout_ms = timeout_ms;
  return c;
}

// --- boot ------------------------------------------------------------------

void test_boot_state_forbids_motion() {
  // THE FAILURE: ESP powers up with the Pi not yet booted. If the watchdog
  // defaulted to "healthy until proven otherwise", the motors would be live
  // during the Pi's ~30 s boot with nothing supervising them.
  LinkWatchdog w(cfg());
  TEST_ASSERT_FALSE(w.motionAllowed());
  TEST_ASSERT_EQUAL(LinkHealth::NeverSeen, w.health());
}

void test_boot_state_survives_ticking_without_frames() {
  // Ticking for a long time with no Pi must not somehow age into "allowed".
  LinkWatchdog w(cfg());
  for (uint32_t t = 0; t <= 30000; t += 20) {
    TEST_ASSERT_FALSE(w.update(t));  // no edge: the link was never up
    TEST_ASSERT_FALSE(w.motionAllowed());
  }
  TEST_ASSERT_EQUAL(LinkHealth::NeverSeen, w.health());
}

void test_never_seen_reports_max_age() {
  LinkWatchdog w(cfg());
  TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, w.sinceFeedMs(5000));
}

void test_first_frame_enables_motion() {
  LinkWatchdog w(cfg());
  w.feed(1000);
  TEST_ASSERT_TRUE(w.motionAllowed());
  TEST_ASSERT_EQUAL(LinkHealth::Ok, w.health());
}

// --- steady state ----------------------------------------------------------

void test_regular_frames_never_trip() {
  // 50 Hz heartbeat against a 100 ms timeout: 30 s of normal operation.
  LinkWatchdog w(cfg());
  for (uint32_t t = 0; t <= 30000; t += 20) {
    w.feed(t);
    TEST_ASSERT_FALSE(w.update(t));
    TEST_ASSERT_TRUE(w.motionAllowed());
  }
}

void test_jitter_below_timeout_does_not_trip() {
  // THE FAILURE: nuisance trips. Linux delivering a frame 80 ms late is normal
  // scheduling jitter, not a dead brain. Tripping on it makes the robot unusable.
  LinkWatchdog w(cfg());
  w.feed(0);
  const uint32_t gaps[] = {20, 60, 80, 99, 15, 90};
  uint32_t t = 0;
  for (uint32_t g : gaps) {
    t += g;
    TEST_ASSERT_FALSE(w.update(t));
    w.feed(t);
  }
  TEST_ASSERT_TRUE(w.motionAllowed());
}

// --- the trip --------------------------------------------------------------

void test_trips_exactly_at_timeout() {
  LinkWatchdog w(cfg(100));
  w.feed(1000);
  TEST_ASSERT_FALSE(w.update(1099));  // 99 ms — still inside
  TEST_ASSERT_TRUE(w.motionAllowed());
  TEST_ASSERT_TRUE(w.update(1100));   // 100 ms — trips
  TEST_ASSERT_FALSE(w.motionAllowed());
  TEST_ASSERT_EQUAL(LinkHealth::Lost, w.health());
}

void test_edge_delivered_exactly_once() {
  // The caller logs and brakes on the edge. Repeating it every tick would flood
  // the log at 50 Hz and bury whatever else went wrong.
  LinkWatchdog w(cfg());
  w.feed(0);
  int edges = 0;
  for (uint32_t t = 0; t <= 2000; t += 20) {
    if (w.update(t)) ++edges;
  }
  TEST_ASSERT_EQUAL_INT(1, edges);
}

// --- the latch -------------------------------------------------------------

void test_returning_frames_do_not_silently_re_enable_motion() {
  // THE FAILURE — the important one. USB re-enumerates mid-dock. The Pi's world
  // model is now stale by an unknown amount, but it is still mid-sequence and
  // still sending DRIVE. Without the latch the robot resumes a docking manoeuvre
  // based on sensor data it stopped receiving seconds ago.
  LinkWatchdog w(cfg());
  w.feed(0);
  TEST_ASSERT_TRUE(w.update(500));      // trip
  TEST_ASSERT_FALSE(w.motionAllowed());

  for (uint32_t t = 500; t <= 1500; t += 20) {  // traffic returns, healthy rate
    w.feed(t);
    w.update(t);
    TEST_ASSERT_FALSE(w.motionAllowed());        // still latched. Deliberately.
  }
  TEST_ASSERT_EQUAL(LinkHealth::Lost, w.health());
}

void test_resume_works_when_link_is_actually_healthy() {
  LinkWatchdog w(cfg());
  w.feed(0);
  TEST_ASSERT_TRUE(w.update(500));
  w.feed(600);
  TEST_ASSERT_TRUE(w.resume(610));
  TEST_ASSERT_TRUE(w.motionAllowed());
}

void test_resume_refused_while_link_still_dead() {
  // THE FAILURE: a RESUME sitting in the ESP's RX buffer from before the dropout
  // gets parsed after it, and re-enables the motors on a cable that is still out.
  LinkWatchdog w(cfg());
  w.feed(0);
  TEST_ASSERT_TRUE(w.update(500));
  TEST_ASSERT_FALSE(w.resume(500));   // no fresh traffic -> refused
  TEST_ASSERT_FALSE(w.resume(10000));
  TEST_ASSERT_FALSE(w.motionAllowed());
}

void test_resume_refused_before_any_frame() {
  LinkWatchdog w(cfg());
  TEST_ASSERT_FALSE(w.resume(0));
  TEST_ASSERT_FALSE(w.resume(50000));
  TEST_ASSERT_FALSE(w.motionAllowed());
}

void test_resume_is_noop_when_healthy() {
  LinkWatchdog w(cfg());
  w.feed(100);
  TEST_ASSERT_FALSE(w.resume(110));   // nothing latched to clear
  TEST_ASSERT_TRUE(w.motionAllowed());
}

void test_can_trip_again_after_resume() {
  LinkWatchdog w(cfg());
  w.feed(0);
  TEST_ASSERT_TRUE(w.update(500));
  w.feed(600);
  TEST_ASSERT_TRUE(w.resume(610));
  TEST_ASSERT_TRUE(w.update(800));    // second dropout trips like the first
  TEST_ASSERT_FALSE(w.motionAllowed());
}

// --- millis() rollover -----------------------------------------------------

void test_no_false_trip_across_millis_rollover() {
  // THE FAILURE: at 49.7 days of uptime millis() wraps to 0. Written as
  // `now < last + timeout` this brakes a healthy robot; written as unsigned
  // subtraction it just works. Cheap to get right, expensive to debug.
  LinkWatchdog w(cfg(100));
  uint32_t t = UINT32_MAX - 200;
  w.feed(t);
  for (int i = 0; i < 30; ++i) {
    t += 20;  // wraps through zero mid-loop
    TEST_ASSERT_FALSE(w.update(t));
    w.feed(t);
  }
  TEST_ASSERT_TRUE(w.motionAllowed());
}

void test_trips_correctly_across_millis_rollover() {
  LinkWatchdog w(cfg(100));
  w.feed(UINT32_MAX - 50);        // 50 ms before the wrap
  TEST_ASSERT_FALSE(w.update(20));  // 71 ms elapsed across the wrap
  TEST_ASSERT_TRUE(w.update(60));   // 111 ms elapsed -> trips
}

void test_age_correct_across_rollover() {
  LinkWatchdog w(cfg());
  w.feed(UINT32_MAX - 99);
  TEST_ASSERT_EQUAL_UINT32(150, w.sinceFeedMs(50));
}

// ---------------------------------------------------------------------------

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_boot_state_forbids_motion);
  RUN_TEST(test_boot_state_survives_ticking_without_frames);
  RUN_TEST(test_never_seen_reports_max_age);
  RUN_TEST(test_first_frame_enables_motion);
  RUN_TEST(test_regular_frames_never_trip);
  RUN_TEST(test_jitter_below_timeout_does_not_trip);
  RUN_TEST(test_trips_exactly_at_timeout);
  RUN_TEST(test_edge_delivered_exactly_once);
  RUN_TEST(test_returning_frames_do_not_silently_re_enable_motion);
  RUN_TEST(test_resume_works_when_link_is_actually_healthy);
  RUN_TEST(test_resume_refused_while_link_still_dead);
  RUN_TEST(test_resume_refused_before_any_frame);
  RUN_TEST(test_resume_is_noop_when_healthy);
  RUN_TEST(test_can_trip_again_after_resume);
  RUN_TEST(test_no_false_trip_across_millis_rollover);
  RUN_TEST(test_trips_correctly_across_millis_rollover);
  RUN_TEST(test_age_correct_across_rollover);
  return UNITY_END();
}
