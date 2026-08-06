// Dead-man's handle for the Pi 5 control link.
//
// WHY THIS EXISTS
// ---------------
// Once the Raspberry Pi becomes the brain, every wheel command arrives over USB
// serial from a machine running Linux. Linux is not real-time: it takes page
// faults, resets USB buses, blocks on the SD card, and pauses for garbage
// collection. The Pi WILL go quiet occasionally, and no amount of tuning removes
// that — it is a property of the platform, not a bug to be fixed.
//
// The failure that matters is not "the Pi crashed". It is "the Pi went quiet
// while the last command it sent was DRIVE FORWARD". Motor drivers hold their
// last commanded value indefinitely; they have no opinion about whether anyone is
// still steering. A silent Pi therefore means a robot that keeps going, at speed,
// under a trolley, with nothing left to stop it.
//
// So the ESP does not wait to be told to stop. It requires continuous proof that
// someone is still in charge, and stops when that proof stops arriving. Same idea
// as the dead-man's handle on a train: the driver must keep holding it down, and
// releasing it — deliberately, or by slumping over — applies the brakes.
//
// WHY THE LATCH
// -------------
// When the link recovers, motion does NOT resume on its own. A dropout means the
// Pi's picture of the world is now stale by an unknown amount: it has been
// commanding a docking sequence against sensor data it stopped receiving. Letting
// it silently pick up where it left off means acting on that stale picture. The
// latch forces a deliberate RESUME, which is the Pi asserting it has re-read the
// world and is ready to drive again.
//
// This is the same shape as SafetyMonitor's E-stop latch, and for the same reason.
#pragma once

#include <stdint.h>

namespace tb {

struct LinkConfig {
  // No valid frame for this long -> the link is considered lost.
  //
  // Sized as ~5 missed ticks at the 50 Hz control rate. Long enough to ride out
  // ordinary Linux scheduling jitter without nuisance trips; short enough that a
  // robot at 300 mm/s travels only ~30 mm before the brakes go on.
  uint32_t timeout_ms = 100;
};

enum class LinkHealth : uint8_t {
  NeverSeen,  // booted, no frame has EVER arrived — the Pi may not be running yet
  Ok,         // a frame arrived recently
  Lost        // frames stopped; latched until an explicit resume()
};

class LinkWatchdog {
public:
  LinkWatchdog() = default;
  explicit LinkWatchdog(const LinkConfig& cfg) : cfg_(cfg) {}

  // A well-formed frame arrived from the Pi. Call on EVERY valid frame.
  //
  // Note this does not clear a latched Lost: it only records that the link is
  // carrying traffic again, which is what lets a later resume() succeed.
  void feed(uint32_t now_ms);

  // Advance the check. Returns true EXACTLY ONCE, on the transition into Lost,
  // so the caller can log/act on the edge without repeating itself every tick.
  // (Same edge convention as StallDetector::update.)
  bool update(uint32_t now_ms);

  // Clear a latched Lost. Refuses — returning false — unless the link is
  // currently carrying fresh traffic, so a RESUME sent down a cable that is still
  // unplugged cannot re-enable the motors.
  bool resume(uint32_t now_ms);

  // The single question main.cpp asks before commanding any motion.
  bool motionAllowed() const { return health_ == LinkHealth::Ok; }

  LinkHealth health() const { return health_; }
  const char* healthName() const;

  // Milliseconds since the last frame; UINT32_MAX if none has ever arrived.
  uint32_t sinceFeedMs(uint32_t now_ms) const;

private:
  LinkConfig cfg_{};
  LinkHealth health_ = LinkHealth::NeverSeen;  // boot state disallows motion
  bool seen_ = false;
  uint32_t last_feed_ = 0;
};

} // namespace tb
