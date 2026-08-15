#pragma once

#include <Adafruit_PWMServoDriver.h>
#include <Arduino.h>

#include "HardwareConfig.h"

using MotorAbortCallback = bool (*)();

class MotorController {
 public:
  MotorController();

  void begin(Stream &log);
  bool flipXAxis(MotorAbortCallback shouldAbort = nullptr);
  bool flipYAxis(MotorAbortCallback shouldAbort = nullptr);
  bool extractXAxis(MotorAbortCallback shouldAbort = nullptr);
  bool extractYAxis(MotorAbortCallback shouldAbort = nullptr);
  bool resetAllToHome(MotorAbortCallback shouldAbort = nullptr);
  void stopAll();
  bool setXAxisFlipped(bool flipped, MotorAbortCallback shouldAbort = nullptr);
  bool setYAxisFlipped(bool flipped, MotorAbortCallback shouldAbort = nullptr);
  bool isXAxisFlipped() const;
  bool isYAxisFlipped() const;
  bool areBothAxesFlipped() const;

 private:
  Adafruit_PWMServoDriver pwm_;
  int servoAngle_[SERVO_COUNT];
  bool xAxisFlipped_;
  bool yAxisFlipped_;

  // Is the cached flip state still TRUE OF THE HARDWARE?
  //
  // xAxisFlipped_/yAxisFlipped_ record what we last commanded, and the setters
  // skip the move when the request already matches. That is a useful shortcut
  // right up to the moment the servos stop obeying us -- stopAll() cuts PWM on
  // all four channels, and an aborted ramp leaves them part-way. In both cases
  // the flags still claim a position the arms are no longer holding, and the
  // next command silently does nothing because it thinks it has already run.
  //
  // Cleared whenever we give up control; set again only after a completed move.
  bool flipStateKnown_;

  bool moveServoPairSlow(uint8_t servo1, uint8_t servo2, int targetAngle,
                         MotorAbortCallback shouldAbort);
  void setServoAngle(uint8_t channel, int angle);
  void setServoPulse(uint8_t channel, int pulseUs);
};
