// The clamp actuator behind its H-bridge (BTS7960). Open/close drive the two PWM
// directions; currentAmps() exposes the current sense used for stall detection.
#pragma once

namespace fal {

struct IClamp {
  virtual ~IClamp() = default;

  // Drive open / close at speed in [0, 1].
  virtual void open(float speed) = 0;
  virtual void close(float speed) = 0;

  // Coast the clamp (PWM to zero).
  virtual void stop() = 0;

  // R_EN/L_EN enable line.
  virtual void enable(bool on) = 0;

  // Current through the active half-bridge, in amps (for over-current / stall).
  virtual float currentAmps() const = 0;
};

} // namespace fal
