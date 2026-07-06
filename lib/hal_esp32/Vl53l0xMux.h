// IAlignmentSensor backend: N × VL53L0X behind a TCA9548A I²C mux.
//
// The mux isolates the sensors, so every sensor can keep its default address 0x29;
// we just select a channel before talking to each. This is the DEFAULT backend
// because the sensor count can grow with zero GPIO/firmware pin changes.
//
// Uses the popular pololu/VL53L0X driver. Out-of-range / no-return reads are flagged
// invalid (not dropped) so the interpreter can reason about gaps in a caged underside.
#pragma once

#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "IAlignmentSensor.h"
#include "pins.h"

namespace tb {

class Vl53l0xMux : public IAlignmentSensor {
public:
  Vl53l0xMux(uint8_t mux_addr, const uint8_t* channels, size_t count)
      : mux_addr_(mux_addr), count_(count > kMaxZones ? kMaxZones : count) {
    for (size_t i = 0; i < count_; ++i) channels_[i] = channels[i];
  }

  bool begin() override {
    bool ok = true;
    for (size_t i = 0; i < count_; ++i) {
      select(channels_[i]);
      sensors_[i].setTimeout(200);
      if (!sensors_[i].init()) {
        ok = false;
        continue;
      }
      sensors_[i].setMeasurementTimingBudget(20000);  // 20 ms
    }
    return ok;
  }

  AlignmentFrame read() override {
    AlignmentFrame f;
    f.zone_count = count_;
    f.t_ms = millis();
    for (size_t i = 0; i < count_; ++i) {
      select(channels_[i]);
      const uint16_t mm = sensors_[i].readRangeSingleMillimeters();
      f.zones[i].mm = mm;
      f.zones[i].valid = !sensors_[i].timeoutOccurred() && mm < 8000;
    }
    return f;
  }

  size_t zoneCount() const override { return count_; }

private:
  void select(uint8_t ch) {
    Wire.beginTransmission(mux_addr_);
    Wire.write(static_cast<uint8_t>(1u << ch));
    Wire.endTransmission();
  }

  uint8_t mux_addr_;
  uint8_t channels_[kMaxZones] = {0};
  size_t count_;
  VL53L0X sensors_[kMaxZones];
};

} // namespace tb
