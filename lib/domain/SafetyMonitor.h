// The single authority for whether the clamp may close, and whether the robot
// must stop. The docking state machine can only actuate the clamp *through* this.
//
// Core principle: the clamp must not begin to close unless alignment reports it is
// safe. Limit switches are irrelevant here — they only confirm travel.
#pragma once

#include "types.h"

namespace tb {

class SafetyMonitor {
public:
  // Refresh with the latest alignment estimate and hardware fault flags.
  // An asserted E-stop latches until explicitly cleared.
  void update(const AlignmentState& align, const FaultFlags& faults);

  // The interlock: may the clamp begin/continue closing right now?
  bool clampCloseAllowed() const;

  // Any condition that demands the drivetrain stop immediately.
  bool safeStopRequired() const;

  bool estopLatched() const { return estop_latch_; }
  void clearEstopLatch() { estop_latch_ = false; }

private:
  AlignmentState align_{};
  FaultFlags faults_{};
  bool estop_latch_ = false;
};

} // namespace tb
