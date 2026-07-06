// Serial telemetry + operator command port. The adapter owns the wire format
// (JSON out via ArduinoJson, command parsing via SerialCommands); the domain only
// publishes a snapshot, logs a line, and polls for the next Command.
#pragma once

#include "types.h"

namespace tb {

struct ITelemetry {
  virtual ~ITelemetry() = default;

  // Emit a status snapshot (adapter decides the format).
  virtual void publish(const char* state, const bool* corner_present, size_t n_corners,
                       const Pose2D& pose, bool confirmed, const FaultFlags& faults) = 0;

  // Emit a human-readable log line.
  virtual void log(const char* msg) = 0;

  // Return the next pending operator command (Command::None if nothing pending).
  virtual Command poll() = 0;
};

} // namespace tb
