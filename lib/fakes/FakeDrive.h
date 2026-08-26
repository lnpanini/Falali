// Host test double for IDrive — records the last motion command and gang lines.
#pragma once

#include "IDrive.h"

namespace fal {

class FakeDrive : public IDrive {
public:
  void move(const DriveCommand& cmd) override {
    last_ = cmd;
    ++move_count_;
    stopped_ = false;
  }
  void stop() override {
    last_ = DriveCommand{};
    stopped_ = true;
  }
  void enable(bool on) override { enabled_ = on; }
  void brake(bool on) override { braked_ = on; }
  bool fault() const override { return fault_; }

  // Test inspection / injection.
  const DriveCommand& last() const { return last_; }
  int moveCount() const { return move_count_; }
  bool stopped() const { return stopped_; }
  bool enabled() const { return enabled_; }
  bool braked() const { return braked_; }
  void setFault(bool f) { fault_ = f; }

private:
  DriveCommand last_{};
  int move_count_ = 0;
  bool stopped_ = false;
  bool enabled_ = false;
  bool braked_ = false;
  bool fault_ = false;
};

} // namespace fal
