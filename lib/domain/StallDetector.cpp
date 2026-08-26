#include "StallDetector.h"

namespace fal {

static inline float absf(float v) { return v < 0.0f ? -v : v; }

void StallDetector::reset(uint32_t now_ms) {
  tripped_ = false;
  tripped_at_cmd_ = 0;
  stalling_ = false;
  stall_since_ = now_ms;
  grace_until_ = now_ms;
}

void StallDetector::noteCommandIncrease(uint32_t now_ms) {
  grace_until_ = now_ms + cfg_.grace_ms;
  stalling_ = false;  // restart the window; the motor is being asked to speed up
}

bool StallDetector::update(uint32_t now_ms, int cmd, float rpm) {
  if (tripped_) return false;  // edge already delivered

  // Not commanded hard enough to expect motion -> nothing to police.
  if (cmd < cfg_.break_away_cmd) { stalling_ = false; return false; }

  // Inside the post-step grace window -> acceleration, not a stall.
  if ((int32_t)(now_ms - grace_until_) < 0) { stalling_ = false; return false; }

  const bool moving = absf(rpm) >= cfg_.rpm_floor;
  if (moving) { stalling_ = false; return false; }

  if (!stalling_) { stalling_ = true; stall_since_ = now_ms; return false; }

  if (now_ms - stall_since_ >= cfg_.trip_ms) {
    tripped_ = true;
    tripped_at_cmd_ = cmd;
    return true;
  }
  return false;
}

} // namespace fal
