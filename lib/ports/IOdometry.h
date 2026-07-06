// Planar odometry port. update() advances the estimate each control tick; the
// implementation decides how (dead-reckoning from the command, or wheel pulses).
// Kept behind a port so the source can be swapped without touching docking logic.
#pragma once

#include "types.h"

namespace tb {

struct IOdometry {
  virtual ~IOdometry() = default;

  virtual Pose2D pose() const = 0;
  virtual void reset() = 0;

  // Advance the estimate for this tick. `latest_cmd` is the motion just commanded
  // (used by dead-reckoning; a wheel-pulse impl may ignore it).
  virtual void update(const DriveCommand& latest_cmd, uint32_t now_ms) = 0;
};

} // namespace tb
