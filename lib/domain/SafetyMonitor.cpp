#include "SafetyMonitor.h"

namespace tb {

void SafetyMonitor::update(const AlignmentState& align, const FaultFlags& faults) {
  align_ = align;
  faults_ = faults;
  if (faults.estop) estop_latch_ = true;  // latch — never auto-clear
}

bool SafetyMonitor::safeStopRequired() const {
  return faults_.motor_alarm || faults_.clamp_overcurrent || faults_.estop ||
         estop_latch_;
}

bool SafetyMonitor::clampCloseAllowed() const {
  return align_.clamp_safe && !faults_.motor_alarm && !faults_.clamp_overcurrent &&
         !estop_latch_;
}

} // namespace tb
