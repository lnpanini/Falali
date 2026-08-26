// The bench rig's only automatic fault protection.
//
// The BLD-120A exposes no ALM output and no FG, and the bench PSU has no current
// limit — so before the AS5600 was fitted there was literally no signal from which
// a stall could be inferred. A locked rotor held at the driver's limit sinks up to
// 8 A at 24 V (~192 W) into stationary windings, and the 10 A fuse never opens
// because 8 A sits below it indefinitely.
//
// The trip keys on the actual fault — "commanded to move, not moving" — rather
// than on current, so it does not false-trip on legitimate inrush.
#pragma once

#include <stdint.h>

namespace fal {

struct StallConfig {
  // Below this command the motor is expected to be still, so zero RPM is normal
  // rather than a fault. Set from the measured break-away point.
  int      break_away_cmd = 40;
  float    rpm_floor      = 5.0f;   // |RPM| under this counts as "not turning"
  uint32_t trip_ms        = 250;    // sustained stall before tripping
  uint32_t grace_ms       = 300;    // suppression after a command increase
};

class StallDetector {
public:
  StallDetector() = default;
  explicit StallDetector(const StallConfig& cfg) : cfg_(cfg) {}

  // Clears the trip and restarts all timing. Call before every routine.
  void reset(uint32_t now_ms);

  // Opens a grace window. Call whenever the command steps UP, so ordinary
  // acceleration from rest is not mistaken for a locked rotor.
  void noteCommandIncrease(uint32_t now_ms);

  // Returns true ONCE, on the transition into the tripped state.
  bool update(uint32_t now_ms, int cmd, float rpm);

  bool tripped() const { return tripped_; }
  int  trippedAtCmd() const { return tripped_at_cmd_; }

private:
  StallConfig cfg_{};
  bool     tripped_        = false;
  int      tripped_at_cmd_ = 0;
  bool     stalling_       = false;  // currently inside a candidate stall
  uint32_t stall_since_    = 0;
  uint32_t grace_until_    = 0;
};

} // namespace fal
