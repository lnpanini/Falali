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

namespace fal {

class Bts7960Clamp : public IClamp {
public:
  Bts7960Clamp(uint8_t rpwm, uint8_t rpwm_channel, uint8_t lpwm, uint8_t lpwm_channel,
               uint8_t en, uint8_t is_close, uint8_t is_open, float amps_per_volt = 8.5f)
      : rpwm_(rpwm), rpwm_ch_(rpwm_channel), lpwm_(lpwm), lpwm_ch_(lpwm_channel), en_(en),
        is_close_(is_close), is_open_(is_open), k_(amps_per_volt), active_is_(is_close) {}

  // Present only when every pin is wired. On the Wheel Drive PCB the clamp lives
  // on ESP-ARM, so all pins are kNoPin and every method below no-ops rather than
  // driving GPIOs that belong to the rear-right wheel.
  bool present() const {
    return rpwm_ != pins::kNoPin && lpwm_ != pins::kNoPin && en_ != pins::kNoPin;
  }

  void begin() {
    if (!present()) return;
    pinMode(en_, OUTPUT);
    digitalWrite(en_, HIGH);  // R_EN/L_EN active-high
    rpwm_pwm_.begin(rpwm_, rpwm_ch_, pins::kPwmFreqHz, pins::kPwmResBits);
    lpwm_pwm_.begin(lpwm_, lpwm_ch_, pins::kPwmFreqHz, pins::kPwmResBits);
    stop();
  }

  void close(float speed) override {
    if (!present()) return;
    lpwm_pwm_.writeDuty(0);
    rpwm_pwm_.writeFraction(speed);
    active_is_ = is_close_;
  }
  void open(float speed) override {
    if (!present()) return;
    rpwm_pwm_.writeDuty(0);
    lpwm_pwm_.writeFraction(speed);
    active_is_ = is_open_;
  }
  void stop() override {
    if (!present()) return;
    rpwm_pwm_.writeDuty(0);
    lpwm_pwm_.writeDuty(0);
  }
  void enable(bool on) override {
    if (!present()) return;
    digitalWrite(en_, on ? HIGH : LOW);
  }

  // Guarded on the IS pin itself, not on present(): the sense pins are separate
  // from the drive pins, so a board could wire the H-bridge without them.
  //
  // Without this, readFaults() called analogReadMilliVolts(0xFF) every control
  // tick on the Wheel Drive PCB, where the whole clamp is kNoPin. The ADC driver
  // rejected it and logged five lines each time -- a flood that swamped USB-CDC
  // and buried the boot banner (2026-08-11).
  //
  // Returning 0 A is honest here: there is no clamp on this board, so it cannot
  // be drawing current. The SafetyMonitor comparison against kClampStallAmps
  // then simply never trips, which is the correct behaviour for absent hardware.
  float currentAmps() const override {
    if (active_is_ == pins::kNoPin) return 0.0f;
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

} // namespace fal
