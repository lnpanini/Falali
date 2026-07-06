// Host tests for SafetyMonitor — the single clamp-authority interlock.
#include <unity.h>

#include "SafetyMonitor.h"

using namespace tb;

void setUp() {}
void tearDown() {}

void test_allows_clamp_when_confirmed() {
  SafetyMonitor sm;
  sm.update(true, FaultFlags{});
  TEST_ASSERT_TRUE(sm.clampCloseAllowed());
  TEST_ASSERT_FALSE(sm.safeStopRequired());
}

void test_denies_clamp_when_not_confirmed() {
  SafetyMonitor sm;
  sm.update(false, FaultFlags{});  // alignment not confirmed
  TEST_ASSERT_FALSE(sm.clampCloseAllowed());
}

void test_motor_alarm_denies_and_requires_stop() {
  SafetyMonitor sm;
  FaultFlags f;
  f.motor_alarm = true;
  sm.update(true, f);
  TEST_ASSERT_FALSE(sm.clampCloseAllowed());
  TEST_ASSERT_TRUE(sm.safeStopRequired());
}

void test_overcurrent_denies_and_requires_stop() {
  SafetyMonitor sm;
  FaultFlags f;
  f.clamp_overcurrent = true;
  sm.update(true, f);
  TEST_ASSERT_FALSE(sm.clampCloseAllowed());
  TEST_ASSERT_TRUE(sm.safeStopRequired());
}

// E-stop latches: releasing the button does not resume; only an explicit clear does.
void test_estop_latches_until_cleared() {
  SafetyMonitor sm;
  FaultFlags es;
  es.estop = true;
  sm.update(true, es);
  TEST_ASSERT_TRUE(sm.safeStopRequired());
  TEST_ASSERT_TRUE(sm.estopLatched());

  sm.update(true, FaultFlags{});  // button released
  TEST_ASSERT_TRUE(sm.safeStopRequired());  // still latched
  TEST_ASSERT_FALSE(sm.clampCloseAllowed());

  sm.clearEstopLatch();
  sm.update(true, FaultFlags{});
  TEST_ASSERT_FALSE(sm.safeStopRequired());
  TEST_ASSERT_TRUE(sm.clampCloseAllowed());
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_allows_clamp_when_confirmed);
  RUN_TEST(test_denies_clamp_when_not_confirmed);
  RUN_TEST(test_motor_alarm_denies_and_requires_stop);
  RUN_TEST(test_overcurrent_denies_and_requires_stop);
  RUN_TEST(test_estop_latches_until_cleared);
  return UNITY_END();
}
