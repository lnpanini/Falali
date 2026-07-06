// Host tests for DockingStateMachine — the sequence, and its safety-gated clamp.
#include <unity.h>

#include "DockingStateMachine.h"
#include "FakeClamp.h"
#include "FakeClock.h"
#include "FakeDrive.h"
#include "FakeLimitSwitches.h"
#include "SafetyMonitor.h"

using namespace tb;

namespace {

// A test rig wiring the state machine to fakes, with a single-call tick helper.
struct Rig {
  FakeDrive drive;
  FakeClamp clamp;
  FakeLimitSwitches limits;
  FakeClock clk;
  SafetyMonitor safety;
  DockingConfig cfg;
  DockingStateMachine sm{drive, clamp, limits, safety, clk, cfg};

  void tick(const AlignmentState& a, const FaultFlags& f) {
    safety.update(a, f);  // main loop refreshes safety before the SM tick
    sm.update(a);
  }
};

AlignmentState under() {
  AlignmentState a;
  a.under_trolley = 0.6f;
  return a;
}

AlignmentState clampSafe() {
  AlignmentState a;
  a.under_trolley = 0.95f;
  a.centred = 0.95f;
  a.fresh = true;
  a.clamp_safe = true;
  return a;
}

// Drive the sequence from Idle to Clamping (gate open the whole way).
void toClamping(Rig& r) {
  r.sm.handleCommand(Command::Dock);       // -> Entering
  r.tick(under(), FaultFlags{});           // under detected -> Aligning
  r.tick(clampSafe(), FaultFlags{});       // clamp_safe -> ReadyToClamp
  r.tick(clampSafe(), FaultFlags{});       // gate open -> Clamping
}

} // namespace

void setUp() {}
void tearDown() {}

void test_happy_path_reaches_clamped_only_after_gate() {
  Rig r;
  r.sm.handleCommand(Command::Dock);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Entering);
  TEST_ASSERT_TRUE(r.drive.enabled());

  r.tick(under(), FaultFlags{});
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Aligning);

  r.tick(clampSafe(), FaultFlags{});
  TEST_ASSERT_TRUE(r.sm.state() == DockState::ReadyToClamp);

  r.tick(clampSafe(), FaultFlags{});
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Clamping);

  r.tick(clampSafe(), FaultFlags{});  // still closing, switch not tripped
  TEST_ASSERT_TRUE(r.clamp.action() == ClampAction::Close);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Clamping);

  r.limits.setClosed(true);           // closed switch confirms travel
  r.tick(clampSafe(), FaultFlags{});
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Clamped);
  TEST_ASSERT_TRUE(r.clamp.action() == ClampAction::Stop);
}

// Aligning strafes opposite the lateral bias (toward centre) and never clamps.
void test_aligning_strafes_and_holds_without_gate() {
  Rig r;
  r.sm.handleCommand(Command::Dock);
  r.tick(under(), FaultFlags{});
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Aligning);

  AlignmentState off;
  off.under_trolley = 0.9f;
  off.centred = 0.4f;
  off.lateral = 0.5f;      // right-heavy
  off.clamp_safe = false;
  r.tick(off, FaultFlags{});
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Aligning);
  TEST_ASSERT_TRUE(r.drive.last().vy < 0.0f);  // strafe left, back toward centre
}

// Losing the gate at READY_TO_CLAMP recovers to Aligning — it must not clamp.
void test_lost_gate_recovers_without_clamping() {
  Rig r;
  r.sm.handleCommand(Command::Dock);
  r.tick(under(), FaultFlags{});
  r.tick(clampSafe(), FaultFlags{});
  TEST_ASSERT_TRUE(r.sm.state() == DockState::ReadyToClamp);

  AlignmentState lost;  // alignment collapsed
  lost.clamp_safe = false;
  r.tick(lost, FaultFlags{});
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Aligning);
  TEST_ASSERT_TRUE(r.clamp.action() != ClampAction::Close);
}

// Over-current mid-clamp routes to FAULT (safe stop).
void test_overcurrent_during_clamping_faults() {
  Rig r;
  toClamping(r);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Clamping);

  FaultFlags oc;
  oc.clamp_overcurrent = true;
  r.tick(clampSafe(), oc);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Fault);
  TEST_ASSERT_TRUE(r.drive.braked());
}

// E-stop from any state forces FAULT.
void test_estop_forces_fault() {
  Rig r;
  r.sm.handleCommand(Command::Dock);
  r.tick(under(), FaultFlags{});
  FaultFlags es;
  es.estop = true;
  r.tick(under(), es);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Fault);
}

// Abort stops everything and returns to Idle.
void test_abort_returns_to_idle() {
  Rig r;
  toClamping(r);
  r.sm.handleCommand(Command::Abort);
  TEST_ASSERT_TRUE(r.sm.state() == DockState::Idle);
  TEST_ASSERT_TRUE(r.drive.stopped());
}

int main(int argc, char** argv) {
  (void)argc;
  (void)argv;
  UNITY_BEGIN();
  RUN_TEST(test_happy_path_reaches_clamped_only_after_gate);
  RUN_TEST(test_aligning_strafes_and_holds_without_gate);
  RUN_TEST(test_lost_gate_recovers_without_clamping);
  RUN_TEST(test_overcurrent_during_clamping_faults);
  RUN_TEST(test_estop_forces_fault);
  RUN_TEST(test_abort_returns_to_idle);
  return UNITY_END();
}
