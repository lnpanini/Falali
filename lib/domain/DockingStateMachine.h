// Orchestrates the geometric docking sequence over the 4 corner sensors + odometry.
// Actuates the drivetrain/clamp only through ports, drives the odometry estimate as
// it moves, and can only engage the clamp through the SafetyMonitor gate.
//
//   IDLE -> APPROACH -> ORIENT -> CENTER_X -> CENTER_Y -> CONFIRM -> CLAMP_ENGAGE
//        -> CLAMPED -> UNCLAMPING -> IDLE ;  FAULT is sticky, reachable anywhere.
//
// CLAMP_ENGAGE is the handoff boundary to the (externally built) arm/servo clamp
// subsystem — here it is a placeholder close through IClamp.
#pragma once

#include "IClamp.h"
#include "IClock.h"
#include "IDrive.h"
#include "ILimitSwitches.h"
#include "IOdometry.h"
#include "SafetyMonitor.h"
#include "types.h"

namespace tb {

enum class DockState : uint8_t {
  Idle,
  Approach,
  Orient,
  CenterX,
  CenterY,
  Confirm,
  ClampEngage,
  Clamped,
  Unclamping,
  Fault
};

struct DockingConfig {
  float approach_speed = 0.25f;  // vx while driving under
  float rotate_speed = 0.20f;    // omega while orienting
  float centre_speed = 0.20f;    // vx/vy while centring
  float front_offset_mm = 0.0f;  // platform-centre offset behind the front sensor line
  float side_offset_mm = 0.0f;   // platform-centre offset from the side sensor line
  float centre_tol_mm = 8.0f;    // "centred" tolerance
  float clamp_speed = 0.60f;
  uint32_t confirm_hold_ms = 300;
  uint32_t approach_timeout_ms = 8000;
  uint32_t orient_timeout_ms = 8000;
  uint32_t center_timeout_ms = 12000;
  uint32_t clamp_timeout_ms = 6000;
  uint32_t unclamp_timeout_ms = 6000;
};

class DockingStateMachine {
public:
  DockingStateMachine(IDrive& drive, IClamp& clamp, ILimitSwitches& limits, IOdometry& odom,
                      SafetyMonitor& safety, IClock& clock, const DockingConfig& cfg);

  void handleCommand(Command c);

  // Advance one control tick. `present[Corner]` is the debounced board-present state
  // per corner. The caller must refresh the SafetyMonitor beforehand.
  void update(const bool present[kNumCorners]);

  DockState state() const { return state_; }
  const char* stateName() const;
  const char* lastReason() const { return last_reason_; }
  bool alignmentConfirmed() const { return confirmed_; }

private:
  void enter(DockState s);
  void toFault(const char* reason);
  void applyMove(const DriveCommand& cmd, uint32_t now);  // drive + odometry
  void applyStop(uint32_t now);

  IDrive& drive_;
  IClamp& clamp_;
  ILimitSwitches& limits_;
  IOdometry& odom_;
  SafetyMonitor& safety_;
  IClock& clock_;
  DockingConfig cfg_;

  DockState state_ = DockState::Idle;
  uint32_t state_since_ms_ = 0;
  const char* last_reason_ = "";
  bool confirmed_ = false;

  // Centring bookkeeping.
  float x_near_ = 0.0f, x_far_ = 0.0f, x_target_ = 0.0f;
  bool x_far_found_ = false;
  uint8_t y_phase_ = 0;
  float y_a_ = 0.0f, y_b_ = 0.0f, y_target_ = 0.0f;
  bool confirm_holding_ = false;
  uint32_t confirm_since_ms_ = 0;
};

} // namespace tb
