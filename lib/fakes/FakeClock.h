// Host test double for IClock — time only moves when a test advances it.
#pragma once

#include "IClock.h"

namespace fal {

class FakeClock : public IClock {
public:
  uint32_t millis() const override { return now_; }

  void set(uint32_t ms) { now_ = ms; }
  void advance(uint32_t ms) { now_ += ms; }

private:
  uint32_t now_ = 0;
};

} // namespace fal
