// Host test double for IAlignmentSensor — returns frames scripted by the test,
// including dropouts, noise and invalid returns to model a caged/barred underside.
#pragma once

#include <deque>

#include "IAlignmentSensor.h"

namespace tb {

class FakeAlignmentSensor : public IAlignmentSensor {
public:
  explicit FakeAlignmentSensor(size_t zone_count = 2) : zone_count_(zone_count) {}

  bool begin() override { return true; }
  size_t zoneCount() const override { return zone_count_; }

  AlignmentFrame read() override {
    if (!queue_.empty()) {
      last_ = queue_.front();
      queue_.pop_front();
    }
    return last_;
  }

  // Queue a full frame to be returned on the next read().
  void push(const AlignmentFrame& f) { queue_.push_back(f); }

private:
  size_t zone_count_;
  std::deque<AlignmentFrame> queue_;
  AlignmentFrame last_{};
};

} // namespace tb
