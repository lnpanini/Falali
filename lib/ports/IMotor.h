// A single BLDC wheel motor behind its driver (BLD120A).
// Maps to the driver's SV (speed), F/R (direction), EN, BRK and ALARM lines.
#pragma once

namespace tb {

struct IMotor {
  virtual ~IMotor() = default;

  // Signed speed in [-1, 1]: magnitude -> SV duty, sign -> F/R direction.
  virtual void setSpeed(float speed) = 0;

  // EN line — false coasts the motor to a stop.
  virtual void enable(bool on) = 0;

  // BRK line — true actively brakes.
  virtual void brake(bool on) = 0;

  // ALARM line (active) — driver-reported fault (over-current, Hall error, ...).
  virtual bool fault() const = 0;
};

} // namespace tb
