// Host tests for DeadReckonOdometry — commanded-velocity integration.
#include <unity.h>

#include "DeadReckonOdometry.h"

using namespace fal;

namespace {
OdometryCal cleanCal() {
  OdometryCal c;
  c.max_lin_mm_s = 1000.0f;  // 1 unit command for 1 s -> 1000 mm
  c.max_ang_rad_s = 1.0f;    // 1 unit command for 1 s -> 1 rad
  return c;
}
} // namespace

void setUp() {}
void tearDown() {}

// Pure forward moves x, not y or theta.
void test_forward_increases_x() {
  DeadReckonOdometry o(cleanCal());
  o.update(DriveCommand{}, 0);            // first tick primes the time base
  o.update({1.0f, 0.0f, 0.0f}, 1000);     // vx=1 for 1 s
  const Pose2D p = o.pose();
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, p.x_mm);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, p.y_mm);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, p.theta_rad);
}

// Pure strafe moves y.
void test_strafe_increases_y() {
  DeadReckonOdometry o(cleanCal());
  o.update(DriveCommand{}, 0);
  o.update({0.0f, 1.0f, 0.0f}, 1000);     // vy=1 for 1 s
  const Pose2D p = o.pose();
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 1000.0f, p.y_mm);
  TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, p.x_mm);
}

// Pure rotation changes theta.
void test_rotate_changes_theta() {
  DeadReckonOdometry o(cleanCal());
  o.update(DriveCommand{}, 0);
  o.update({0.0f, 0.0f, 1.0f}, 1000);     // omega=1 for 1 s
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, o.pose().theta_rad);
}

// reset() returns to the origin.
void test_reset() {
  DeadReckonOdometry o(cleanCal());
  o.update(DriveCommand{}, 0);
  o.update({1.0f, 0.0f, 0.0f}, 1000);
  o.reset();
  const Pose2D p = o.pose();
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, p.x_mm);
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_forward_increases_x);
  RUN_TEST(test_strafe_increases_y);
  RUN_TEST(test_rotate_changes_theta);
  RUN_TEST(test_reset);
  return UNITY_END();
}
