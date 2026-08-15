#include "Bts7960Motor.h"

Bts7960Motor::Bts7960Motor()
    : pins_{-1, -1, -1, -1, false},
      duty_(0),
      configured_(false),
      direction_(Bts7960Direction::Stopped) {}

void Bts7960Motor::begin(const Bts7960Pins &pins, uint8_t duty, Stream &log) {
  pins_ = pins;
  duty_ = duty;
  direction_ = Bts7960Direction::Stopped;
  configured_ = hasValidPins(pins_);

  if (!configured_) {
    log.println("BTS7960 motor disabled: fill all pin constants first.");
    return;
  }

  pinMode(pins_.rpwmPin, OUTPUT);
  pinMode(pins_.lpwmPin, OUTPUT);
  pinMode(pins_.rEnablePin, OUTPUT);
  pinMode(pins_.lEnablePin, OUTPUT);

  digitalWrite(pins_.rEnablePin, HIGH);
  digitalWrite(pins_.lEnablePin, HIGH);
  stop();
}

void Bts7960Motor::extend() {
  if (!configured_) {
    return;
  }

  if (pins_.directionInverted) {
    writePwm(0, duty_);
  } else {
    writePwm(duty_, 0);
  }

  direction_ = Bts7960Direction::Extending;
}

void Bts7960Motor::retract() {
  if (!configured_) {
    return;
  }

  if (pins_.directionInverted) {
    writePwm(duty_, 0);
  } else {
    writePwm(0, duty_);
  }

  direction_ = Bts7960Direction::Retracting;
}

void Bts7960Motor::stop() {
  if (configured_) {
    writePwm(0, 0);
  }

  direction_ = Bts7960Direction::Stopped;
}

void Bts7960Motor::setDuty(uint8_t duty) {
  duty_ = duty;
}

bool Bts7960Motor::isConfigured() const {
  return configured_;
}

bool Bts7960Motor::isRunning() const {
  return direction_ != Bts7960Direction::Stopped;
}

Bts7960Direction Bts7960Motor::direction() const {
  return direction_;
}

bool Bts7960Motor::hasValidPins(const Bts7960Pins &pins) const {
  return pins.rpwmPin >= 0 && pins.lpwmPin >= 0 && pins.rEnablePin >= 0 &&
         pins.lEnablePin >= 0;
}

void Bts7960Motor::writePwm(int extendDuty, int retractDuty) {
  analogWrite(pins_.rpwmPin, extendDuty);
  analogWrite(pins_.lpwmPin, retractDuty);
}
