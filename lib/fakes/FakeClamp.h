// Host test double for IClamp — records the last actuation and reports injected current.
#pragma once

#include "IClamp.h"
#include "types.h"

namespace tb {

class FakeClamp : public IClamp {
public:
  void open(float speed) override {
    action_ = ClampAction::Open;
    speed_ = speed;
  }
  void close(float speed) override {
    action_ = ClampAction::Close;
    speed_ = speed;
  }
  void stop() override {
    action_ = ClampAction::Stop;
    speed_ = 0.0f;
  }
  void enable(bool on) override { enabled_ = on; }
  float currentAmps() const override { return current_; }

  // Test inspection / injection.
  ClampAction action() const { return action_; }
  float speed() const { return speed_; }
  bool enabled() const { return enabled_; }
  void setCurrent(float a) { current_ = a; }

private:
  ClampAction action_ = ClampAction::Stop;
  float speed_ = 0.0f;
  bool enabled_ = false;
  float current_ = 0.0f;
};

} // namespace tb
