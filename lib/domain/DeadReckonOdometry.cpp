#include <cmath>

#include "DeadReckonOdometry.h"

namespace tb {

void DeadReckonOdometry::update(const DriveCommand& cmd, uint32_t now_ms) {
  if (!have_last_) {  // first tick just establishes the time base
    last_ms_ = now_ms;
    have_last_ = true;
    return;
  }
  const uint32_t elapsed = now_ms - last_ms_;
  last_ms_ = now_ms;
  if (elapsed == 0) return;

  const float dt = static_cast<float>(elapsed) / 1000.0f;
  const float lin = cal_.max_lin_mm_s * dt;

  // Body-frame displacement this tick, rotated into the world frame.
  const float bx = cmd.vx * lin;
  const float by = cmd.vy * lin;
  const float c = std::cos(pose_.theta_rad);
  const float s = std::sin(pose_.theta_rad);

  pose_.x_mm += bx * c - by * s;
  pose_.y_mm += bx * s + by * c;
  pose_.theta_rad += cmd.omega * cal_.max_ang_rad_s * dt;
}

} // namespace tb
