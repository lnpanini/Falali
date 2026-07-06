// The single authority for whether the clamp may engage, and whether the robot
// must stop. The docking state machine can only actuate the clamp *through* this.
//
// Core principle: the clamp must not engage unless alignment is CONFIRMED centred.
// Limit switches are irrelevant here — they only confirm clamp travel.
#pragma once

#include "types.h"

namespace tb {

class SafetyMonitor {
public:
  // Refresh with the latest alignment verdict and hardware fault flags.
  // An asserted E-stop latches until explicitly cleared.
  void update(bool alignment_confirmed, const FaultFlags& faults);

  // The interlock: may the clamp engage right now?
  bool clampCloseAllowed() const;

  // Any condition that demands the drivetrain stop immediately.
  bool safeStopRequired() const;

  bool estopLatched() const { return estop_latch_; }
  void clearEstopLatch() { estop_latch_ = false; }

private:
  bool alignment_confirmed_ = false;
  FaultFlags faults_{};
  bool estop_latch_ = false;
};

} // namespace tb
