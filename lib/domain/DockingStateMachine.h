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

namespace fal {

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
  // +1 or -1. Which way Orient turns to bring the lagging leading-corner onto
  // the edge, and it CANNOT be derived here: it depends on the physical corner
  // layout and on whether the caller rotated the frame (DockFrame.h's axis swap
  // is a reflection, which inverts it). Get it wrong and Orient turns away from
  // square, loses the corner, turns back, and hunts until orient_timeout_ms.
  //
  // The failure looks like "it pivots endlessly", so if you see that, flip this
  // before suspecting the sensors.
  float rotate_dir = 1.0f;
  float centre_speed = 0.20f;    // vx/vy while centring
  float front_offset_mm = 0.0f;  // platform-centre offset behind the front sensor line

  // CENTRE ON OPPOSED EDGES INSTEAD OF ONE SENSOR PAIR CROSSING BOTH.
  //
  // The default method marks both edges with the LEADING pair: it enters the
  // near edge, then exits the far edge, and the midpoint of those two robot
  // positions sits half a sensor span short of the platform centre -- which is
  // what front_offset_mm exists to add back.
  //
  // The opposed method instead marks
  //     the TRAILING pair covering the near edge, and
  //     the LEADING  pair clearing  the far  edge,
  // which are symmetric about the platform centre, so their midpoint IS the
  // centre and no offset is needed.
  //
  // WHY THAT IS WORTH A FLAG. front_offset_mm is a REAL millimetre value
  // subtracted from an ODOMETRY-space position, so it is only correct while
  // OdometryCal::max_lin_mm_s is correct -- and a dead-reckoned speed constant
  // is the least trustworthy number in this system (estimates for this platform
  // have ranged 1155-1657 mm/s). Removing the offset removes that dependence
  // entirely: both marks and the current position scale together, so the robot
  // parks at the true centre whatever the constant says.
  //
  // Optical overreach cancels under BOTH methods -- one mark is early and the
  // other late by the same grazing distance either way. This buys scale
  // independence, not edge accuracy.
  //
  // Requires the platform to be deeper than the sensor span, so that all four
  // corners are covered at once; faults out if the trailing pair never covers.
  bool centre_opposed_pairs = false;
  float side_offset_mm = 0.0f;   // platform-centre offset from the side sensor line
  float centre_tol_mm = 8.0f;    // "centred" tolerance
  // Run the SECOND centring axis at all. Some geometries can only correct one:
  // on Falali the trolley's own wheels block travel along the lateral axis
  // once the base is underneath, so CenterY would command motion the chassis
  // physically cannot make and simply time out. Skipping it goes CenterX ->
  // Confirm, handing the remaining axis to the arm's own sensors.
  //
  // Defaults true because that is the general case; the platform that cannot do
  // it should be the one to say so.
  bool centre_lateral = true;
  float clamp_speed = 0.60f;
  uint32_t confirm_hold_ms = 300;
  // Confirm used to be the ONLY state without a deadline, so a geometry that
  // cannot present all four corners at once left it waiting forever with the
  // drivetrain stopped and nothing on the console (observed 2026-08-13). Every
  // other state faults out; this one silently did not.
  uint32_t confirm_timeout_ms = 5000;
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

  // The three marks the travel-axis bisection is built from, exposed so a
  // composition root can PRINT them. Without these, diagnosing a bad park means
  // inferring x_far by algebra from a state-transition pose -- which works, but
  // only if every other constant is already known to be right. When the
  // complaint is "it overshot by 10 cm", these say whether the edges were found
  // in the wrong place or the drive failed to reach a correct target.
  float markNear() const { return x_near_; }
  float markFar() const { return x_far_; }
  float markTarget() const { return x_target_; }
  bool markFarFound() const { return x_far_found_; }
  float markLo() const { return x_lo_; }
  bool markLoFound() const { return x_lo_found_; }

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
  // Trailing pair covering the near edge -- the opposed-method partner to
  // x_far_. See DockingConfig::centre_opposed_pairs.
  float x_lo_ = 0.0f;
  bool x_far_found_ = false;
  bool x_lo_found_ = false;
  uint8_t y_phase_ = 0;
  float y_a_ = 0.0f, y_b_ = 0.0f, y_target_ = 0.0f;
  bool confirm_holding_ = false;
  uint32_t confirm_since_ms_ = 0;
};

} // namespace fal
