// Orchestrates the docking/clamping sequence. Actuates the drivetrain and clamp
// only through ports, and can only close the clamp via the SafetyMonitor gate.
//
//   IDLE -> ENTERING -> ALIGNING -> READY_TO_CLAMP -> CLAMPING -> CLAMPED
//        -> UNCLAMPING -> IDLE ;  FAULT is a sticky safe state reachable anywhere.
#pragma once

#include "IClamp.h"
#include "IClock.h"
#include "IDrive.h"
#include "ILimitSwitches.h"
#include "SafetyMonitor.h"
#include "types.h"

namespace tb {

enum class DockState : uint8_t {
  Idle,
  Entering,
  Aligning,
  ReadyToClamp,
  Clamping,
  Clamped,
  Unclamping,
  Fault
};

struct DockingConfig {
  float enter_speed = 0.25f;            // vx while driving under the trolley
  float centre_kp = 0.8f;               // strafe gain applied to lateral bias
  float max_strafe = 0.40f;             // clamp on |vy| during aligning
  float clamp_speed = 0.60f;            // clamp open/close duty
  float enter_under_threshold = 0.50f;  // under_trolley needed to leave ENTERING
  uint32_t enter_timeout_ms = 8000;
  uint32_t align_timeout_ms = 8000;
  uint32_t clamp_timeout_ms = 6000;
  uint32_t unclamp_timeout_ms = 6000;
};

class DockingStateMachine {
public:
  DockingStateMachine(IDrive& drive, IClamp& clamp, ILimitSwitches& limits,
                      SafetyMonitor& safety, IClock& clock, const DockingConfig& cfg);

  // Apply an operator command (from serial).
  void handleCommand(Command c);

  // Advance the sequence one control tick. The caller must have refreshed the
  // SafetyMonitor with the same alignment/faults beforehand.
  void update(const AlignmentState& align);

  DockState state() const { return state_; }
  const char* stateName() const;
  const char* lastReason() const { return last_reason_; }

private:
  void enter(DockState s);
  void toFault(const char* reason);

  IDrive& drive_;
  IClamp& clamp_;
  ILimitSwitches& limits_;
  SafetyMonitor& safety_;
  IClock& clock_;
  DockingConfig cfg_;

  DockState state_ = DockState::Idle;
  uint32_t state_since_ms_ = 0;
  const char* last_reason_ = "";
};

} // namespace tb
