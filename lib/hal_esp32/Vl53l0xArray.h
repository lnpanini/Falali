// IAlignmentSensor backend: N × VL53L0X on one I²C bus, distinguished by XSHUT
// address re-assignment (no mux). One dedicated GPIO per sensor holds it in reset
// until we wake it and move it off the shared default address 0x29.
//
// Same port as Vl53l0xMux — the docking logic cannot tell which backend is in use.
#pragma once

#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "IAlignmentSensor.h"

namespace tb {

class Vl53l0xArray : public IAlignmentSensor {
public:
  Vl53l0xArray(const uint8_t* xshut_pins, size_t count, uint8_t base_addr = 0x30)
      : count_(count > kMaxZones ? kMaxZones : count), base_(base_addr) {
    for (size_t i = 0; i < count_; ++i) xshut_[i] = xshut_pins[i];
  }

  bool begin() override {
    // Hold every sensor in reset first.
    for (size_t i = 0; i < count_; ++i) {
      pinMode(xshut_[i], OUTPUT);
      digitalWrite(xshut_[i], LOW);
    }
    delay(10);

    bool ok = true;
    for (size_t i = 0; i < count_; ++i) {
      digitalWrite(xshut_[i], HIGH);  // wake exactly one sensor (still at 0x29)
      delay(10);
      sensors_[i].setTimeout(200);
      if (!sensors_[i].init()) {
        ok = false;
        continue;
      }
      sensors_[i].setAddress(static_cast<uint8_t>(base_ + i));  // move it off 0x29
      sensors_[i].setMeasurementTimingBudget(20000);
    }
    return ok;
  }

  AlignmentFrame read() override {
    AlignmentFrame f;
    f.zone_count = count_;
    f.t_ms = millis();
    for (size_t i = 0; i < count_; ++i) {
      const uint16_t mm = sensors_[i].readRangeSingleMillimeters();
      f.zones[i].mm = mm;
      f.zones[i].valid = !sensors_[i].timeoutOccurred() && mm < 8000;
    }
    return f;
  }

  size_t zoneCount() const override { return count_; }

private:
  size_t count_;
  uint8_t base_;
  uint8_t xshut_[kMaxZones] = {0};
  VL53L0X sensors_[kMaxZones];
};

} // namespace tb
