// Version-agnostic single LEDC PWM output for Arduino-ESP32.
//
// Core 3.x uses the pin-based API (ledcAttach/ledcWrite(pin,...)); core 2.x uses
// the channel-based API (ledcSetup + ledcAttachPin + ledcWrite(channel,...)). This
// shim hides the difference so the adapters build on either core.
#pragma once

#include <Arduino.h>

namespace tb {

class PwmPin {
public:
  void begin(uint8_t pin, uint8_t channel, uint32_t freq_hz, uint8_t res_bits) {
    pin_ = pin;
    channel_ = channel;
    res_bits_ = res_bits;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(pin_, freq_hz, res_bits_);
#else
    ledcSetup(channel_, freq_hz, res_bits_);
    ledcAttachPin(pin_, channel_);
#endif
    writeDuty(0);
  }

  void writeDuty(uint32_t duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pin_, duty);
#else
    ledcWrite(channel_, duty);
#endif
  }

  // Write a 0..1 fraction of full scale.
  void writeFraction(float f) {
    if (f < 0.0f) f = 0.0f;
    else if (f > 1.0f) f = 1.0f;
    const uint32_t max_duty = (1u << res_bits_) - 1u;
    writeDuty(static_cast<uint32_t>(f * max_duty));
  }

private:
  uint8_t pin_ = 0;
  uint8_t channel_ = 0;
  uint8_t res_bits_ = 8;
};

} // namespace tb
