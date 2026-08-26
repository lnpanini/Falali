// Serial telemetry + operator command port. The adapter owns the wire format
// (JSON out via ArduinoJson, command parsing via SerialCommands); the domain only
// publishes a snapshot, logs a line, and polls for the next Command.
#pragma once

#include "types.h"

namespace fal {

struct ITelemetry {
  virtual ~ITelemetry() = default;

  // Emit a status snapshot (adapter decides the format).
  // `link` is the control-link health name (see LinkWatchdog::healthName) — the Pi
  // needs to see the ESP's own view of the link, not just infer it from silence.
  virtual void publish(const char* state, const bool* corner_present, size_t n_corners,
                       const Pose2D& pose, bool confirmed, const FaultFlags& faults,
                       const char* link) = 0;

  // Emit a human-readable log line.
  virtual void log(const char* msg) = 0;

  // Return the next pending operator command (Command::None if nothing pending).
  virtual Command poll() = 0;
};

} // namespace fal
