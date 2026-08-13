#pragma once
// Operator-facing "the platform is aligned" signal.
//
// WHY THERE IS NO LED HERE BY DEFAULT
// -----------------------------------
// The DevKitC-1's onboard RGB LED is on GPIO48, and on the Wheel Drive PCB
// GPIO48 is FR's motor ENABLE line (pins.h kWheelEN[1]). Driving a WS2812 frame
// on that pin fires an 800 kHz burst of high pulses into the front-right
// driver's enable input -- HIGH is ASSERTED through the inverting adapters. The
// BLD-120A very likely filters pulses that short, but "probably ignored" is not
// a basis for pointing a data stream at a motor enable.
//
// So the default indicator is a telemetry line. Pass a real GPIO to `pin` and it
// also blinks -- GPIO3 or GPIO14 on this board.
//
// NOT GPIO1 or GPIO2, which this comment used to recommend: they were the unused
// analog-encoder fallback then, and they are the ESP-ARM UART link now.
//
// Edge-triggered, not level: it announces the TRANSITION into and out of
// alignment. A message every control tick would be noise, and the moment
// alignment is achieved or lost is the only part an operator can act on.
#include <Arduino.h>

#include "ITelemetry.h"
#include "pins.h"

namespace tb {

class AlignmentIndicator {
 public:
  AlignmentIndicator(ITelemetry& telemetry, uint8_t pin = pins::kNoPin,
                     uint32_t blink_ms = 150)
      : telemetry_(telemetry), pin_(pin), blink_ms_(blink_ms) {}

  void begin() {
    if (pin_ == pins::kNoPin) return;
    pinMode(pin_, OUTPUT);
    digitalWrite(pin_, LOW);
  }

  void update(uint32_t now_ms, bool aligned) {
    if (aligned != aligned_) {
      aligned_ = aligned;
      telemetry_.log(aligned ? "ALIGNED — platform centred within tolerance"
                             : "alignment lost");
      last_toggle_ms_ = now_ms;
      level_ = aligned;
      write(level_);
    }

    // Flash while aligned, so it reads as a live state rather than a stuck pin.
    if (aligned_ && pin_ != pins::kNoPin && now_ms - last_toggle_ms_ >= blink_ms_) {
      last_toggle_ms_ = now_ms;
      level_ = !level_;
      write(level_);
    }
  }

  bool aligned() const { return aligned_; }

 private:
  void write(bool on) {
    if (pin_ != pins::kNoPin) digitalWrite(pin_, on ? HIGH : LOW);
  }

  ITelemetry& telemetry_;
  uint8_t pin_;
  uint32_t blink_ms_;
  uint32_t last_toggle_ms_ = 0;
  bool aligned_ = false;
  bool level_ = false;
};

}  // namespace tb
