// Host test double for ILimitSwitches — states are set directly by the test.
#pragma once

#include "ILimitSwitches.h"

namespace fal {

class FakeLimitSwitches : public ILimitSwitches {
public:
  void update() override { ++update_count_; }
  bool clampOpen() const override { return open_; }
  bool clampClosed() const override { return closed_; }

  // Test injection.
  void setOpen(bool v) { open_ = v; }
  void setClosed(bool v) { closed_ = v; }
  int updateCount() const { return update_count_; }

private:
  bool open_ = false;
  bool closed_ = false;
  int update_count_ = 0;
};

} // namespace fal
