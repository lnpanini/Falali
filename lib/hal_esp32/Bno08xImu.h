#pragma once
// BNO08x heading over I2C, 4-wire hookup (3V3 / GND / SDA / SCL).
//
// ADDRESS IS 0x4B, NOT 0x4A. The breakout on this robot has ADR strapped high
// (confirmed by bus scan 2026-08-11). 0x4A is the more common default, so a
// future board may differ -- the address is a constructor argument for exactly
// that reason.
//
// NO INT, NO RST. The breakout's remaining pins are not wired, which costs two
// things and it is better to state them than discover them:
//
//   1. Reports must be POLLED. Without INT we cannot know when data is ready, so
//      update() asks on every control tick and usually gets nothing. That is
//      normal, not an error.
//   2. Recovery is software-only. If SHTP desynchronises there is no reset line
//      to pull, so the only option is re-running begin(). staleness detection
//      below is what triggers that -- a frozen heading is far more dangerous
//      than an absent one, because the docking logic would keep steering by it.
//
// The project's own Pi-5 design doc argues for UART-RVC over I2C precisely
// because the ESP32 handles BNO08x clock stretching badly. The fabricated board
// wired I2C, so that recommendation was overtaken by the hardware. Expect
// occasional dropouts and let the staleness watchdog do its job.
#include <Arduino.h>
#include <Wire.h>

#include <Adafruit_BNO08x.h>

#include "IImu.h"

namespace tb {

class Bno08xImu : public IImu {
 public:
  explicit Bno08xImu(uint8_t addr = 0x4B, uint32_t stale_ms = 500)
      : bno_(-1), addr_(addr), stale_ms_(stale_ms) {}

  bool begin() override {
    // reset_pin -1: the library falls back to an SHTP software reset.
    if (!bno_.begin_I2C(addr_, &Wire)) {
      valid_ = false;
      return false;
    }
    // 100 Hz. Faster buys nothing -- the control loop runs slower than this and
    // every extra report is bus traffic shared with four ToF sensors.
    if (!bno_.enableReport(SH2_ROTATION_VECTOR, 10000)) {
      valid_ = false;
      return false;
    }
    last_report_ms_ = millis();
    valid_ = true;
    return true;
  }

  void update(uint32_t now_ms) override {
    sh2_SensorValue_t v;
    while (bno_.getSensorEvent(&v)) {
      if (v.sensorId != SH2_ROTATION_VECTOR) continue;
      yaw_deg_ = quatToYawDeg(v.un.rotationVector.real, v.un.rotationVector.i,
                              v.un.rotationVector.j, v.un.rotationVector.k);
      last_report_ms_ = now_ms;
      valid_ = true;
    }

    // A HEADING THAT STOPS UPDATING MUST NOT LOOK LIKE A HEADING.
    // yaw_deg_ keeps its last value here on purpose -- callers that ignore
    // valid() get the last real measurement rather than a garbage 0.0 -- but
    // valid() going false is the signal that it can no longer be trusted.
    if (now_ms - last_report_ms_ > stale_ms_) valid_ = false;
  }

  float yawDeg() const override { return yaw_deg_; }
  bool valid() const override { return valid_; }

  // Re-run begin(). The only recovery available without a reset pin.
  bool recover() { return begin(); }

 private:
  // Yaw about Z from the rotation-vector quaternion. Only the heading term is
  // needed; roll and pitch are dropped because the robot is planar.
  static float quatToYawDeg(float w, float x, float y, float z) {
    const float siny = 2.0f * (w * z + x * y);
    const float cosy = 1.0f - 2.0f * (y * y + z * z);
    return atan2f(siny, cosy) * 57.2957795f;
  }

  Adafruit_BNO08x bno_;
  uint8_t addr_;
  uint32_t stale_ms_;
  uint32_t last_report_ms_ = 0;
  float yaw_deg_ = 0.0f;
  bool valid_ = false;
};

}  // namespace tb
