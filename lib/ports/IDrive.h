// The drivetrain as a whole, in body-frame commands. A mecanum adapter mixes
// (vx, vy, omega) down to individual wheel motors; the domain never sees wheels.
#pragma once

#include "types.h"

namespace fal {

struct IDrive {
  virtual ~IDrive() = default;

  // Command normalised body-frame motion (each component in [-1, 1]).
  virtual void move(const DriveCommand& cmd) = 0;

  // Zero all motion but keep the drivers enabled.
  virtual void stop() = 0;

  // Ganged EN line for all wheels (safety cut path).
  virtual void enable(bool on) = 0;

  // Ganged BRK line for all wheels (safety brake path).
  virtual void brake(bool on) = 0;

  // True if any wheel driver reports a fault.
  virtual bool fault() const = 0;
};

} // namespace fal
