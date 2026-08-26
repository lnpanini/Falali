// Host test double for a single IMotor — records the last commanded state.
#pragma once

#include "IMotor.h"

namespace fal {

class FakeMotor : public IMotor {
public:
  void setSpeed(float speed) override { speed_ = speed; }
  void enable(bool on) override { enabled_ = on; }
  void brake(bool on) override { braked_ = on; }
  bool fault() const override { return fault_; }

  // Test inspection / injection.
  float speed() const { return speed_; }
  bool enabled() const { return enabled_; }
  bool braked() const { return braked_; }
  void setFault(bool f) { fault_ = f; }

private:
  float speed_ = 0.0f;
  bool enabled_ = false;
  bool braked_ = false;
  bool fault_ = false;
};

} // namespace fal
