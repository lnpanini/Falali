#include "SafetyMonitor.h"

namespace tb {

void SafetyMonitor::update(bool alignment_confirmed, const FaultFlags& faults) {
  alignment_confirmed_ = alignment_confirmed;
  faults_ = faults;
  if (faults.estop) estop_latch_ = true;  // latch — never auto-clear
}

bool SafetyMonitor::safeStopRequired() const {
  return faults_.motor_alarm || faults_.clamp_overcurrent || faults_.estop ||
         estop_latch_;
}

bool SafetyMonitor::clampCloseAllowed() const {
  return alignment_confirmed_ && !faults_.motor_alarm && !faults_.clamp_overcurrent &&
         !estop_latch_;
}

} // namespace tb
