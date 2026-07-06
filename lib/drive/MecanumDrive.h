// Implements IDrive by mixing body-frame (vx, vy, omega) into four wheel speeds
// and normalising so no wheel is saturated. Pure C++ — composes four IMotor ports,
// so it is exercised natively and reused by the ESP32 build unchanged.
#pragma once

#include "IDrive.h"
#include "IMotor.h"

namespace tb {

class MecanumDrive : public IDrive {
public:
  // Wheel order: front-left, front-right, rear-left, rear-right.
  MecanumDrive(IMotor& fl, IMotor& fr, IMotor& rl, IMotor& rr);

  void move(const DriveCommand& cmd) override;
  void stop() override;
  void enable(bool on) override;
  void brake(bool on) override;
  bool fault() const override;

private:
  IMotor& fl_;
  IMotor& fr_;
  IMotor& rl_;
  IMotor& rr_;
};

} // namespace tb
