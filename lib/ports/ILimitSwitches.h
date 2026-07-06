// Clamp travel limit switches. These confirm mechanical end-of-travel ONLY —
// they never authorise clamping (that is the SafetyMonitor's job).
#pragma once

namespace tb {

struct ILimitSwitches {
  virtual ~ILimitSwitches() = default;

  // Poll / debounce the switches; call once per control tick.
  virtual void update() = 0;

  virtual bool clampOpen() const = 0;    // clamp fully open
  virtual bool clampClosed() const = 0;  // clamp fully closed
};

} // namespace tb
