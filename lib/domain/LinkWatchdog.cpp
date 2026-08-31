#include "LinkWatchdog.h"

namespace fal {

// All elapsed-time maths below is unsigned subtraction on uint32_t, which stays
// correct across the millis() rollover at 2^32 ms (~49.7 days). Writing it as
// `now < last + timeout` instead would break at the wrap and strand the robot
// with either a permanently tripped or permanently blind watchdog.

void LinkWatchdog::feed(uint32_t now_ms) {
  last_feed_ = now_ms;
  seen_ = true;
  // A latched Lost survives incoming traffic — only resume() clears it.
  if (health_ == LinkHealth::NeverSeen) health_ = LinkHealth::Ok;
}

bool LinkWatchdog::update(uint32_t now_ms) {
  // NeverSeen already forbids motion; there is no "edge" to report because the
  // link was never up. Keeping it distinct from Lost makes the telemetry honest:
  // "the Pi has not started yet" is a different problem from "the Pi went away".
  if (health_ != LinkHealth::Ok) return false;

  if (now_ms - last_feed_ >= cfg_.timeout_ms) {
    health_ = LinkHealth::Lost;
    return true;  // deliver the edge exactly once
  }
  return false;
}

bool LinkWatchdog::resume(uint32_t now_ms) {
  if (health_ != LinkHealth::Lost) return false;  // nothing latched to clear
  if (!seen_) return false;
  // The decisive check: is the link actually alive RIGHT NOW? A RESUME that
  // arrived over a healthy link proves the link is healthy; one replayed from a
  // buffer, or sent while the cable is out, does not.
  if (now_ms - last_feed_ >= cfg_.timeout_ms) return false;
  health_ = LinkHealth::Ok;
  return true;
}

uint32_t LinkWatchdog::sinceFeedMs(uint32_t now_ms) const {
  if (!seen_) return UINT32_MAX;
  return now_ms - last_feed_;
}

const char* LinkWatchdog::healthName() const {
  switch (health_) {
    case LinkHealth::NeverSeen: return "NEVER_SEEN";
    case LinkHealth::Ok:        return "OK";
    case LinkHealth::Lost:      return "LOST";
  }
  return "?";
}

} // namespace fal
