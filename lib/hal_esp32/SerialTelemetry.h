// ITelemetry adapter: JSON status out (ArduinoJson) + command parsing (SerialCommands)
// over USB-CDC. The domain only sees publish()/log()/poll(); the wire format lives here.
#pragma once

#include "ITelemetry.h"

namespace tb {

class SerialTelemetry : public ITelemetry {
public:
  // Register serial command handlers. Call once from setup().
  void begin();

  // Service incoming serial (parse commands). Call every loop iteration.
  void pump();

  void publish(const char* state, const bool* corner_present, size_t n_corners,
               const Pose2D& pose, bool confirmed, const FaultFlags& faults) override;
  void log(const char* msg) override;
  Command poll() override;
};

} // namespace tb
