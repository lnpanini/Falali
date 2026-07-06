// Host test double for ITelemetry — captures publishes/logs and dispenses
// scripted operator commands from a queue.
#pragma once

#include <deque>
#include <string>
#include <vector>

#include "ITelemetry.h"

namespace tb {

class FakeTelemetry : public ITelemetry {
public:
  void publish(const char* state, const AlignmentState& align,
               const FaultFlags& faults) override {
    last_state_ = state ? state : "";
    last_align_ = align;
    last_faults_ = faults;
    ++publish_count_;
  }
  void log(const char* msg) override { logs_.push_back(msg ? msg : ""); }

  Command poll() override {
    if (commands_.empty()) return Command::None;
    Command c = commands_.front();
    commands_.pop_front();
    return c;
  }

  // Test injection / inspection.
  void queueCommand(Command c) { commands_.push_back(c); }
  const std::string& lastState() const { return last_state_; }
  int publishCount() const { return publish_count_; }
  const std::vector<std::string>& logs() const { return logs_; }

private:
  std::deque<Command> commands_;
  std::string last_state_;
  AlignmentState last_align_{};
  FaultFlags last_faults_{};
  int publish_count_ = 0;
  std::vector<std::string> logs_;
};

} // namespace tb
