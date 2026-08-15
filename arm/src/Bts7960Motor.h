#pragma once

#include <Arduino.h>

struct Bts7960Pins {
  int rpwmPin;
  int lpwmPin;
  int rEnablePin;
  int lEnablePin;
  bool directionInverted;
};

enum class Bts7960Direction {
  Stopped,
  Extending,
  Retracting,
};

class Bts7960Motor {
 public:
  Bts7960Motor();

  void begin(const Bts7960Pins &pins, uint8_t duty, Stream &log);
  void extend();
  void retract();
  void stop();
  void setDuty(uint8_t duty);

  bool isConfigured() const;
  bool isRunning() const;
  Bts7960Direction direction() const;

 private:
  Bts7960Pins pins_;
  uint8_t duty_;
  bool configured_;
  Bts7960Direction direction_;

  bool hasValidPins(const Bts7960Pins &pins) const;
  void writePwm(int extendDuty, int retractDuty);
};
