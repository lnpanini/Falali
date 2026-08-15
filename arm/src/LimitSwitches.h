#pragma once

#include <Arduino.h>

constexpr int LIMIT_X1_PIN = 11;
constexpr int LIMIT_X2_PIN = 48;
constexpr int LIMIT_Y1_PIN = 5;
constexpr int LIMIT_Y2_PIN = 13;
constexpr int X_ARM_MIN_PIN = 2;
constexpr int X_ARM_MAX_PIN = 42;
constexpr int Y_ARM_MIN_PIN = 3;
constexpr int Y_ARM_MAX_PIN = 1;

struct LimitSwitchStates {
  bool limitX1Pressed;
  bool limitX2Pressed;
  bool limitY1Pressed;
  bool limitY2Pressed;
  bool xArmMinPressed;
  bool xArmMaxPressed;
  bool yArmMinPressed;
  bool yArmMaxPressed;
};

class LimitSwitches {
 public:
  void begin() {
    pinMode(LIMIT_X1_PIN, INPUT_PULLUP);
    pinMode(LIMIT_X2_PIN, INPUT_PULLUP);
    pinMode(LIMIT_Y1_PIN, INPUT_PULLUP);
    pinMode(LIMIT_Y2_PIN, INPUT_PULLUP);
    pinMode(X_ARM_MIN_PIN, INPUT_PULLUP);
    pinMode(X_ARM_MAX_PIN, INPUT_PULLUP);
    pinMode(Y_ARM_MIN_PIN, INPUT_PULLUP);
    pinMode(Y_ARM_MAX_PIN, INPUT_PULLUP);
  }

  LimitSwitchStates readAll() const {
    LimitSwitchStates states;
    states.limitX1Pressed = isLimitX1Pressed();
    states.limitX2Pressed = isLimitX2Pressed();
    states.limitY1Pressed = isLimitY1Pressed();
    states.limitY2Pressed = isLimitY2Pressed();
    states.xArmMinPressed = isXArmMinPressed();
    states.xArmMaxPressed = isXArmMaxPressed();
    states.yArmMinPressed = isYArmMinPressed();
    states.yArmMaxPressed = isYArmMaxPressed();
    return states;
  }

  bool isLimitX1Pressed() const { return isPressed(LIMIT_X1_PIN); }
  bool isLimitX2Pressed() const { return isPressed(LIMIT_X2_PIN); }
  bool isLimitY1Pressed() const { return isPressed(LIMIT_Y1_PIN); }
  bool isLimitY2Pressed() const { return isPressed(LIMIT_Y2_PIN); }
  bool isXArmMinPressed() const { return isPressed(X_ARM_MIN_PIN); }
  bool isXArmMaxPressed() const { return isPressed(X_ARM_MAX_PIN); }
  bool isYArmMinPressed() const { return isPressed(Y_ARM_MIN_PIN); }
  bool isYArmMaxPressed() const { return isPressed(Y_ARM_MAX_PIN); }

  void printStates(Stream &log) const {
    printStates(readAll(), log);
  }

  void printStates(const LimitSwitchStates &states, Stream &log) const {
    printOne(log, "Limit X1", states.limitX1Pressed);
    printOne(log, "Limit X2", states.limitX2Pressed);
    printOne(log, "Limit Y1", states.limitY1Pressed);
    printOne(log, "Limit Y2", states.limitY2Pressed);
    printOne(log, "X Arm Min", states.xArmMinPressed);
    printOne(log, "X Arm Max", states.xArmMaxPressed);
    printOne(log, "Y Arm Min", states.yArmMinPressed);
    printOne(log, "Y Arm Max", states.yArmMaxPressed);
  }

 private:
  bool isPressed(int pin) const {
    return digitalRead(pin) == LOW;
  }

  void printOne(Stream &log, const char *name, bool pressed) const {
    log.print(name);
    log.print(": ");
    log.println(pressed ? "PRESSED" : "OPEN");
  }
};
