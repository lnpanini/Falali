// Host tests for DockingStateMachine — the geometric 4-corner sequence, driven by
// scripted corner presence + odometry, and its safety-gated clamp handoff.
#include <unity.h>

#include "DockingStateMachine.h"
#include "FakeClamp.h"
#include "FakeClock.h"
#include "FakeDrive.h"
#include "FakeLimitSwitches.h"
#include "FakeOdometry.h"
#include "SafetyMonitor.h"

using namespace fal;

namespace {

// Corner-presence patterns [FL, FR, RL, RR].
const bool NONE[4] = {false, false, false, false};
const bool FL_ONLY[4] = {true, false, false, false};
const bool FR_ONLY[4] = {false, true, false, false};
const bool FRONT[4] = {true, true, false, false};
const bool ALL[4] = {true, true, true, true};
const bool FRONT_LOST[4] = {false, false, true, true};   // front off board, rear on
const bool LEFT_LOST[4] = {false, true, false, true};    // FL,RL off -> left edge crossed
const bool RIGHT_LOST[4] = {true, false, true, false};   // FR,RR off -> right edge crossed

struct Rig {
  FakeDrive drive;
  FakeClamp clamp;
  FakeLimitSwitches limits;
  FakeOdometry odom;
  FakeClock clk;
  SafetyMonitor safety;
  DockingConfig cfg;
  DockingStateMachine sm{drive, clamp, limits, odom, safety, clk, cfg};

  void tick(const bool* p, const FaultFlags& f = FaultFlags{}) {
    safety.update(sm.alignmentConfirmed(), f);  // main refreshes the gate first
    sm.update(p);
  }
};

// Drive the full geometric sequence from Idle to the CLAMP_ENGAGE handoff.
void driveToClampEngage(Rig& r) {
  r.sm.handleCommand(Command::Dock);        // -> APPROACH
  r.tick(NONE);                             // no edge yet
  r.tick(FL_ONLY);                          // front corner sees edge -> ORIENT
  r.odom.setX(0);
  r.tick(FRONT);                            // both front -> x_near=0, CENTER_X
  r.odom.setX(200);
  r.tick(FRONT_LOST);                       // far edge -> x_far=200, target=100
  r.odom.setX(100);
  r.tick(ALL);                              // at x target -> CENTER_Y
  r.odom.setY(0);
  r.tick(ALL);                              // strafing left (left still on board)
  r.odom.setY(50);
  r.tick(LEFT_LOST);                        // left edge -> y_a=50
  r.odom.setY(-50);
  r.tick(RIGHT_LOST);                       // right edge -> y_b=-50, target=0
  r.odom.setY(0);
  r.tick(ALL);                              // at y target -> CONFIRM
  r.clk.set(0);
  r.tick(ALL);                              // start confirm hold
  r.clk.set(400);
  r.tick(ALL);                              // hold elapsed -> confirmed -> CLAMP_ENGAGE
}

} // namespace

void setUp() {}
void tearDown() {}

// The sequence visits each phase in order and only clamps at the end.
void test_phase_progression_to_clamped() {
  Rig r;
  r.sm.handleCommand(Command::Dock);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Approach);
  TEST_ASSERT_TRUE(r.drive.enabled());

  r.tick(NONE);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Approach);
  TEST_ASSERT_TRUE(r.drive.last().vx > 0.0f);  // driving under

  r.tick(FL_ONLY);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Orient);

  r.odom.setX(0);
  r.tick(FRONT);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::CenterX);

  r.odom.setX(200);
  r.tick(FRONT_LOST);
  r.odom.setX(100);
  r.tick(ALL);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::CenterY);

  r.odom.setY(0);
  r.tick(ALL);
  r.odom.setY(50);
  r.tick(LEFT_LOST);
  r.odom.setY(-50);
  r.tick(RIGHT_LOST);
  r.odom.setY(0);
  r.tick(ALL);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Confirm);

  // Clamp must NOT have engaged anywhere before CONFIRM.
  TEST_ASSERT_TRUE(r.clamp.action() != ClampAction::Close);
  TEST_ASSERT_FALSE(r.sm.alignmentConfirmed());

  r.clk.set(0);
  r.tick(ALL);
  r.clk.set(400);
  r.tick(ALL);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::ClampEngage);
  TEST_ASSERT_TRUE(r.sm.alignmentConfirmed());

  r.tick(ALL);
  TEST_ASSERT_TRUE(r.clamp.action() == ClampAction::Close);
  r.limits.setClosed(true);
  r.tick(ALL);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Clamped);
  TEST_ASSERT_TRUE(r.clamp.action() == ClampAction::Stop);
}

// Orientation rotates toward the lagging corner — opposite sign for FL-first vs FR-first.
void test_orient_rotates_toward_lagging_corner() {
  Rig rl;
  rl.sm.handleCommand(Command::Dock);
  rl.tick(FL_ONLY);  // -> Orient
  rl.tick(FL_ONLY);  // rotate (FL leads)
  const float wl = rl.drive.last().omega;

  Rig rr;
  rr.sm.handleCommand(Command::Dock);
  rr.tick(FR_ONLY);  // -> Orient
  rr.tick(FR_ONLY);  // rotate (FR leads)
  const float wr = rr.drive.last().omega;

  TEST_ASSERT_TRUE(wl != 0.0f && wr != 0.0f);
  TEST_ASSERT_TRUE((wl < 0.0f) != (wr < 0.0f));  // opposite directions
}

// The clamp only engages after CONFIRM and through the gate.
void test_clamp_engages_only_after_confirm() {
  Rig r;
  driveToClampEngage(r);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::ClampEngage);
  r.tick(ALL);
  TEST_ASSERT_TRUE(r.clamp.action() == ClampAction::Close);
}

// Over-current during the clamp handoff routes to FAULT.
void test_overcurrent_during_clamp_faults() {
  Rig r;
  driveToClampEngage(r);
  FaultFlags oc;
  oc.clamp_overcurrent = true;
  r.tick(ALL, oc);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Fault);
  TEST_ASSERT_TRUE(r.drive.braked());
}

// E-stop from mid-sequence forces FAULT.
void test_estop_forces_fault() {
  Rig r;
  r.sm.handleCommand(Command::Dock);
  r.tick(FL_ONLY);  // Orient
  FaultFlags es;
  es.estop = true;
  r.tick(FL_ONLY, es);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Fault);
}

// Abort stops everything and returns to Idle.
void test_abort_returns_to_idle() {
  Rig r;
  r.sm.handleCommand(Command::Dock);
  r.tick(FL_ONLY);
  r.sm.handleCommand(Command::Abort);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Idle);
  TEST_ASSERT_TRUE(r.drive.stopped());
}


// --- opposed-pair centring ---------------------------------------------------

// With centre_opposed_pairs the near edge is marked by the TRAILING pair (RL,RR)
// covering, and the far edge by the LEADING pair (FL,FR) clearing. Those two
// marks straddle the platform centre, so the target is their bare midpoint --
// no front_offset_mm, hence no dependence on the odometry speed constant.
void test_opposed_pairs_midpoint_needs_no_offset() {
  Rig r;
  r.cfg.centre_opposed_pairs = true;
  r.cfg.front_offset_mm = 0.0f;
  r.cfg.centre_lateral = false;   // one-axis platform, straight to Confirm
  DockingStateMachine sm{r.drive, r.clamp, r.limits, r.odom, r.safety, r.clk, r.cfg};

  sm.handleCommand(Command::Dock);
  r.safety.update(false, FaultFlags{});  sm.update(NONE);
  r.safety.update(false, FaultFlags{});  sm.update(FL_ONLY);
  r.odom.setX(0);
  r.safety.update(false, FaultFlags{});  sm.update(FRONT);       // -> CENTER_X
  TEST_ASSERT_TRUE(sm.state() == DockState::CenterX);

  r.odom.setX(100);
  r.safety.update(false, FaultFlags{});  sm.update(ALL);         // trailing covered
  TEST_ASSERT_TRUE(sm.markLoFound());
  TEST_ASSERT_EQUAL_FLOAT(100.0f, sm.markLo());

  r.odom.setX(300);
  r.safety.update(false, FaultFlags{});  sm.update(FRONT_LOST);  // leading cleared
  TEST_ASSERT_EQUAL_FLOAT(300.0f, sm.markFar());
  // midpoint of the OPPOSED marks, not of near/far
  TEST_ASSERT_EQUAL_FLOAT(200.0f, sm.markTarget());
}

// If the trailing pair never covers, the platform is not deeper than the sensor
// span and the opposed midpoint would be fiction. Fault rather than silently
// falling back to the leading-pair method, which would hide a dead sensor.
void test_opposed_pairs_faults_without_trailing_mark() {
  Rig r;
  r.cfg.centre_opposed_pairs = true;
  DockingStateMachine sm{r.drive, r.clamp, r.limits, r.odom, r.safety, r.clk, r.cfg};

  sm.handleCommand(Command::Dock);
  r.safety.update(false, FaultFlags{});  sm.update(NONE);
  r.safety.update(false, FaultFlags{});  sm.update(FL_ONLY);
  r.odom.setX(0);
  r.safety.update(false, FaultFlags{});  sm.update(FRONT);
  r.odom.setX(300);
  r.safety.update(false, FaultFlags{});  sm.update(FRONT_LOST);  // never saw ALL

  TEST_ASSERT_TRUE(sm.state() == DockState::Fault);
}

// The default must stay the leading-pair method, so existing callers and the
// rest of this file are unaffected by the new flag.
void test_leading_pair_remains_default() {
  Rig r;
  TEST_ASSERT_FALSE(r.cfg.centre_opposed_pairs);
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_phase_progression_to_clamped);
  RUN_TEST(test_orient_rotates_toward_lagging_corner);
  RUN_TEST(test_clamp_engages_only_after_confirm);
  RUN_TEST(test_overcurrent_during_clamp_faults);
  RUN_TEST(test_estop_forces_fault);
  RUN_TEST(test_abort_returns_to_idle);
  RUN_TEST(test_opposed_pairs_midpoint_needs_no_offset);
  RUN_TEST(test_opposed_pairs_faults_without_trailing_mark);
  RUN_TEST(test_leading_pair_remains_default);
  return UNITY_END();
}
