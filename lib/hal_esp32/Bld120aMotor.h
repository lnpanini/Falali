// IMotor adapter for one wheel behind a BLD120A BLDC driver.
//
// SV = speed via LEDC PWM duty; F/R = direction line. EN/BRK/ALARM are typically
// GANGED across all four wheels, so pass the same shared pins to every motor —
// the redundant writes to a shared line are harmless.
//
// Polarities below follow common BLD120A wiring (active-low run/brake, active-low
// ALARM). VERIFY against your board and flip if needed.
#pragma once

#include <Arduino.h>

#include "IMotor.h"
#include "PwmPin.h"
#include "pins.h"

namespace tb {

class Bld120aMotor : public IMotor {
public:
  Bld120aMotor(uint8_t sv, uint8_t sv_channel, uint8_t dir, uint8_t en, uint8_t brk,
               uint8_t alarm)
      : sv_(sv), sv_ch_(sv_channel), dir_(dir), en_(en), brk_(brk), alarm_(alarm) {}

  void begin() {
    pinMode(dir_, OUTPUT);
    pinMode(en_, OUTPUT);
    pinMode(brk_, OUTPUT);
    pinMode(alarm_, INPUT_PULLUP);
    sv_pwm_.begin(sv_, sv_ch_, pins::kPwmFreqHz, pins::kPwmResBits);
    setSpeed(0.0f);
  }

  void setSpeed(float s) override {
    if (s > 1.0f) s = 1.0f;
    else if (s < -1.0f) s = -1.0f;
    const bool reverse = s < 0.0f;
    digitalWrite(dir_, reverse ? LOW : HIGH);  // HIGH = forward (verify)
    sv_pwm_.writeFraction(reverse ? -s : s);
  }

  void enable(bool on) override { digitalWrite(en_, on ? LOW : HIGH); }  // active-low run
  void brake(bool on) override { digitalWrite(brk_, on ? LOW : HIGH); }  // active-low brake
  bool fault() const override { return digitalRead(alarm_) == LOW; }     // active-low ALARM

private:
  uint8_t sv_, sv_ch_, dir_, en_, brk_, alarm_;
  PwmPin sv_pwm_;
};

} // namespace tb
