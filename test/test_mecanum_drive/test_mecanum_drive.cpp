// Host tests for MecanumDrive — mixing and saturation-normalisation.
#include <unity.h>

#include "FakeMotor.h"
#include "MecanumDrive.h"

using namespace tb;

void setUp() {}
void tearDown() {}

// Pure forward: all wheels equal, same sign.
void test_forward_all_equal() {
  FakeMotor fl, fr, rl, rr;
  MecanumDrive d(fl, fr, rl, rr);
  d.move({0.5f, 0.0f, 0.0f});
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, fl.speed());
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, fr.speed());
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, rl.speed());
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f, rr.speed());
}

// Pure strafe: diagonal wheels share signs (mecanum lateral motion).
void test_strafe_signs() {
  FakeMotor fl, fr, rl, rr;
  MecanumDrive d(fl, fr, rl, rr);
  d.move({0.0f, 0.5f, 0.0f});  // vy = +0.5
  TEST_ASSERT_TRUE(fl.speed() < 0.0f);
  TEST_ASSERT_TRUE(fr.speed() > 0.0f);
  TEST_ASSERT_TRUE(rl.speed() > 0.0f);
  TEST_ASSERT_TRUE(rr.speed() < 0.0f);
}

// Combined command that would exceed 1.0 is normalised, preserving ratios.
void test_saturation_normalised() {
  FakeMotor fl, fr, rl, rr;
  MecanumDrive d(fl, fr, rl, rr);
  d.move({1.0f, 1.0f, 1.0f});  // fr = 3.0 before normalise -> peak 3.0
  TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, fr.speed());   // 3.0 / 3.0
  TEST_ASSERT_TRUE(fl.speed() <= 1.0f && fl.speed() >= -1.0f);
  TEST_ASSERT_TRUE(rl.speed() <= 1.0f && rl.speed() >= -1.0f);
  TEST_ASSERT_TRUE(rr.speed() <= 1.0f && rr.speed() >= -1.0f);
}

void test_gang_lines_and_fault() {
  FakeMotor fl, fr, rl, rr;
  MecanumDrive d(fl, fr, rl, rr);
  d.enable(true);
  d.brake(true);
  TEST_ASSERT_TRUE(fl.enabled() && fr.enabled() && rl.enabled() && rr.enabled());
  TEST_ASSERT_TRUE(fl.braked() && fr.braked() && rl.braked() && rr.braked());

  TEST_ASSERT_FALSE(d.fault());
  rr.setFault(true);
  TEST_ASSERT_TRUE(d.fault());
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_forward_all_equal);
  RUN_TEST(test_strafe_signs);
  RUN_TEST(test_saturation_normalised);
  RUN_TEST(test_gang_lines_and_fault);
  return UNITY_END();
}
