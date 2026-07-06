// ILimitSwitches adapter using Bounce2 for debounced reads.
// Switches wire to GND with internal pull-ups, so a pressed (active) switch reads LOW.
#pragma once

#include <Arduino.h>
#include <Bounce2.h>

#include "ILimitSwitches.h"

namespace tb {

class GpioLimitSwitches : public ILimitSwitches {
public:
  GpioLimitSwitches(uint8_t open_pin, uint8_t closed_pin, uint16_t debounce_ms = 10)
      : open_pin_(open_pin), closed_pin_(closed_pin), debounce_ms_(debounce_ms) {}

  void begin() {
    open_.attach(open_pin_, INPUT_PULLUP);
    open_.interval(debounce_ms_);
    closed_.attach(closed_pin_, INPUT_PULLUP);
    closed_.interval(debounce_ms_);
  }

  void update() override {
    open_.update();
    closed_.update();
    open_state_ = (open_.read() == LOW);
    closed_state_ = (closed_.read() == LOW);
  }

  bool clampOpen() const override { return open_state_; }
  bool clampClosed() const override { return closed_state_; }

private:
  uint8_t open_pin_, closed_pin_;
  uint16_t debounce_ms_;
  Bounce open_, closed_;
  bool open_state_ = false;
  bool closed_state_ = false;
};

} // namespace tb
