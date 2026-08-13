#pragma once
// 4x ACS758LCB-050B -> ADS1115 (0x48) -> I2C.
//
// Channel map DISCOVERED ON HARDWARE 2026-08-11, not assumed: each motor was
// driven alone while all four ADC channels were sampled, and the result was a
// clean diagonal in pins.h index order.
//
//     FL -> A0    FR -> A1    RL -> A2    RR -> A3
//
// GAIN IS +/-4.096 V, NOT +/-2.048 V.
// hardware-architecture.md quotes 62.5 uV/LSB, which is the 2.048 V setting, but
// the same section expects 2.44 V at 30 A inrush -- a reading that range would
// clip. Clipping is the one failure a fault signal must not have: a clipped
// sensor cannot tell 5 A from 30 A, the difference between "working hard" and
// "something is badly wrong". One bit of resolution is the right price.
//
// Sensitivity assumes a -050B on a 3.3 V rail. The part is ratiometric, so
// 40 mV/A at 5 V scales to 26.4 mV/A at 3.3 V with a 1.65 V zero point.
// *** IF THE FITTED PARTS ARE -100B OR -050U THESE CONSTANTS ARE WRONG. ***
// -100B halves the sensitivity; -050U moves the zero to ~0.36 V and cannot read
// the negative current BRK produces at all.
#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_ADS1X15.h>

#include "ICurrentSense.h"

namespace tb {

class Ads1115CurrentSense : public ICurrentSense {
 public:
  static constexpr size_t kWheels = 4;

  Ads1115CurrentSense(uint8_t addr = 0x48, float zero_v = 1.65f, float v_per_amp = 0.0264f)
      : addr_(addr), zero_v_(zero_v), v_per_amp_(v_per_amp) {}

  bool begin() override {
    if (!ads_.begin(addr_, &Wire)) {
      valid_ = false;
      return false;
    }
    ads_.setGain(GAIN_ONE);          // +/-4.096 V, 125 uV/LSB
    ads_.setDataRate(RATE_ADS1115_128SPS);
    valid_ = true;
    return true;
  }

  // ONE CHANNEL PER CALL, round-robin.
  //
  // A single-shot conversion at 128 SPS takes ~8 ms. Reading all four inline
  // would block the control loop for 32 ms -- longer than the control period
  // itself, which would wreck loop timing to measure a signal that changes far
  // more slowly. Each wheel is refreshed every 4 ticks instead.
  void update(uint32_t) override {
    if (!valid_) return;
    const int16_t raw = ads_.readADC_SingleEnded(next_);
    const float volts = ads_.computeVolts(raw);
    amps_[next_] = (volts - zero_v_) / v_per_amp_;
    next_ = static_cast<uint8_t>((next_ + 1) % kWheels);
  }

  float amps(size_t wheel) const override {
    return wheel < kWheels ? amps_[wheel] : 0.0f;
  }

  bool valid() const override { return valid_; }

  // Largest magnitude across the four wheels -- what an overcurrent trip wants.
  float peakAmps() const {
    float peak = 0.0f;
    for (size_t i = 0; i < kWheels; ++i) {
      const float a = amps_[i] < 0.0f ? -amps_[i] : amps_[i];
      if (a > peak) peak = a;
    }
    return peak;
  }

 private:
  Adafruit_ADS1115 ads_;
  uint8_t addr_;
  float zero_v_;
  float v_per_amp_;
  float amps_[kWheels] = {0.0f, 0.0f, 0.0f, 0.0f};
  uint8_t next_ = 0;
  bool valid_ = false;
};

}  // namespace tb
