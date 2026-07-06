// IClamp adapter for the clamp actuator behind a BTS7960 (IBT-2) H-bridge.
//
// close() drives RPWM, open() drives LPWM (the other side held low). R_EN/L_EN are
// tied together on one enable line. currentAmps() reads the active side's IS output
// (BTS7960: ~1 V per 8.5 A across the on-board 1k pull-down).
#pragma once

#include <Arduino.h>

#include "IClamp.h"
#include "PwmPin.h"
#include "pins.h"

namespace tb {

class Bts7960Clamp : public IClamp {
public:
  Bts7960Clamp(uint8_t rpwm, uint8_t rpwm_channel, uint8_t lpwm, uint8_t lpwm_channel,
               uint8_t en, uint8_t is_close, uint8_t is_open, float amps_per_volt = 8.5f)
      : rpwm_(rpwm), rpwm_ch_(rpwm_channel), lpwm_(lpwm), lpwm_ch_(lpwm_channel), en_(en),
        is_close_(is_close), is_open_(is_open), k_(amps_per_volt), active_is_(is_close) {}

  void begin() {
    pinMode(en_, OUTPUT);
    digitalWrite(en_, HIGH);  // R_EN/L_EN active-high
    rpwm_pwm_.begin(rpwm_, rpwm_ch_, pins::kPwmFreqHz, pins::kPwmResBits);
    lpwm_pwm_.begin(lpwm_, lpwm_ch_, pins::kPwmFreqHz, pins::kPwmResBits);
    stop();
  }

  void close(float speed) override {
    lpwm_pwm_.writeDuty(0);
    rpwm_pwm_.writeFraction(speed);
    active_is_ = is_close_;
  }
  void open(float speed) override {
    rpwm_pwm_.writeDuty(0);
    lpwm_pwm_.writeFraction(speed);
    active_is_ = is_open_;
  }
  void stop() override {
    rpwm_pwm_.writeDuty(0);
    lpwm_pwm_.writeDuty(0);
  }
  void enable(bool on) override { digitalWrite(en_, on ? HIGH : LOW); }

  float currentAmps() const override {
    const float volts = analogReadMilliVolts(active_is_) / 1000.0f;
    return volts * k_;
  }

private:
  uint8_t rpwm_, rpwm_ch_, lpwm_, lpwm_ch_, en_, is_close_, is_open_;
  float k_;
  uint8_t active_is_;
  PwmPin rpwm_pwm_;
  PwmPin lpwm_pwm_;
};

} // namespace tb
