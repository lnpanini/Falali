// Dead-reckoning odometry: integrates the commanded body velocity over time.
//
// Robust enough for docking centring because we only ever drive to the MIDPOINT
// between two edge events measured by the same integrator — an error in the
// calibration constants scales both marks equally and cancels at the midpoint.
// A wheel-pulse (FG) implementation can replace this behind IOdometry later.
#pragma once

#include "IOdometry.h"

namespace fal {

struct OdometryCal {
  float max_lin_mm_s = 300.0f;  // platform speed at |vx|=|vy|=1
  float max_ang_rad_s = 1.5f;   // yaw rate at |omega|=1
};

class DeadReckonOdometry : public IOdometry {
public:
  explicit DeadReckonOdometry(const OdometryCal& cal) : cal_(cal) {}

  Pose2D pose() const override { return pose_; }

  void reset() override {
    pose_ = Pose2D{};
    have_last_ = false;
  }

  void update(const DriveCommand& cmd, uint32_t now_ms) override;

private:
  OdometryCal cal_;
  Pose2D pose_{};
  uint32_t last_ms_ = 0;
  bool have_last_ = false;
};

} // namespace fal
