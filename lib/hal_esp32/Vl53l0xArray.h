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

  // REQUIRES Wire.begin() TO HAVE BEEN CALLED ALREADY. This class does not own
  // the bus -- it is shared with the IMU, the current-sense ADC and the encoder
  // mux -- so the composition root starts it and every peripheral joins. If the
  // caller forgets, init() talks to an unconfigured peripheral and all four
  // sensors fail identically, which reads exactly like a wiring fault.
  bool begin() override {
    // Hold every sensor in reset first.
    for (size_t i = 0; i < count_; ++i) {
      pinMode(xshut_[i], OUTPUT);
      digitalWrite(xshut_[i], LOW);
      ok_[i] = false;
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
      ok_[i] = true;
    }
    return ok;
  }

  // PER-SENSOR, because begin()'s single bool cannot distinguish "the bus is
  // dead" from "one connector is loose" -- and those need opposite responses.
  // A caller diagnosing hardware wants the three that work, not a global false.
  bool ok(size_t i) const { return i < count_ && ok_[i]; }
  size_t okCount() const {
    size_t n = 0;
    for (size_t i = 0; i < count_; ++i) n += ok_[i] ? 1 : 0;
    return n;
  }

  // Per-sensor offset in mm, added to every VALID reading. Use it to bring four
  // corners onto a common scale when their mounts differ, or to correct genuine
  // sensor error against a known standoff. Signed; zero disables.
  void setOffset(size_t i, int16_t mm) {
    if (i < count_) offset_[i] = mm;
  }
  int16_t offset(size_t i) const { return i < count_ ? offset_[i] : 0; }

  AlignmentFrame read() override {
    AlignmentFrame f;
    f.zone_count = count_;
    f.t_ms = millis();
    for (size_t i = 0; i < count_; ++i) {
      const uint16_t raw = sensors_[i].readRangeSingleMillimeters();
      const bool valid = !sensors_[i].timeoutOccurred() && raw < 8000;
      // OFFSET ONLY A REAL MEASUREMENT. 8190 ("ranged, no target") and 65535
      // ("timeout") are sentinels, not distances -- shifting them would turn a
      // "nothing there" into a plausible reading and silently defeat the band
      // check that CornerEdgeDetector relies on.
      f.zones[i].mm = valid ? withOffset(i, raw) : raw;
      f.zones[i].valid = valid;
    }
    return f;
  }

  size_t zoneCount() const override { return count_; }

private:
  // Clamped to [0, 7999]. The upper clamp matters: without it a large positive
  // offset could push a valid reading past 8000, where read()'s own validity
  // test and every downstream consumer treat it as "no target".
  uint16_t withOffset(size_t i, uint16_t raw) const {
    int32_t v = static_cast<int32_t>(raw) + offset_[i];
    if (v < 0) v = 0;
    if (v > 7999) v = 7999;
    return static_cast<uint16_t>(v);
  }

  size_t count_;
  uint8_t base_;
  int16_t offset_[kMaxZones] = {0};
  uint8_t xshut_[kMaxZones] = {0};
  bool ok_[kMaxZones] = {false};
  VL53L0X sensors_[kMaxZones];
};

} // namespace tb
