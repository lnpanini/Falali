// Host test double for IOdometry — pose is scripted directly by the test.
// update() is a no-op (tests set the pose at simulated edge events).
#pragma once

#include "IOdometry.h"

namespace tb {

class FakeOdometry : public IOdometry {
public:
  Pose2D pose() const override { return pose_; }
  void reset() override {
    pose_ = Pose2D{};
    ++reset_count_;
  }
  void update(const DriveCommand&, uint32_t) override { ++update_count_; }

  // Test injection / inspection.
  void setPose(float x_mm, float y_mm, float theta_rad = 0.0f) {
    pose_ = {x_mm, y_mm, theta_rad};
  }
  void setX(float x_mm) { pose_.x_mm = x_mm; }
  void setY(float y_mm) { pose_.y_mm = y_mm; }
  int updateCount() const { return update_count_; }
  int resetCount() const { return reset_count_; }

private:
  Pose2D pose_{};
  int update_count_ = 0;
  int reset_count_ = 0;
};

} // namespace tb
