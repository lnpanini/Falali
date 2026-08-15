#include "MotorController.h"

MotorController::MotorController()
    : pwm_(PCA9685_ADDRESS), xAxisFlipped_(false), yAxisFlipped_(false),
      flipStateKnown_(false) {
  for (int i = 0; i < SERVO_COUNT; i++) {
    servoAngle_[i] = SERVO_HOME_ANGLE;
  }
}

void MotorController::begin(Stream &log) {
  pwm_.begin();
  pwm_.setOscillatorFrequency(27000000);
  pwm_.setPWMFreq(SERVO_FREQ_HZ);

  delay(10);

  for (uint8_t channel = 0; channel < SERVO_COUNT; channel++) {
    setServoAngle(channel, SERVO_HOME_ANGLE);
  }

  delay(500);

  // begin() drives all four to home and holds them, so the cache is true again.
  xAxisFlipped_ = false;
  yAxisFlipped_ = false;
  flipStateKnown_ = true;

  log.println("Servo control ready.");
}

bool MotorController::flipXAxis(MotorAbortCallback shouldAbort) {
  return setXAxisFlipped(true, shouldAbort);
}

bool MotorController::flipYAxis(MotorAbortCallback shouldAbort) {
  return setYAxisFlipped(true, shouldAbort);
}

bool MotorController::extractXAxis(MotorAbortCallback shouldAbort) {
  return setXAxisFlipped(false, shouldAbort);
}

bool MotorController::extractYAxis(MotorAbortCallback shouldAbort) {
  return setYAxisFlipped(false, shouldAbort);
}

bool MotorController::resetAllToHome(MotorAbortCallback shouldAbort) {
  if (!setXAxisFlipped(false, shouldAbort)) {
    return false;
  }

  return setYAxisFlipped(false, shouldAbort);
}

void MotorController::stopAll() {
  for (uint8_t channel = 0; channel < SERVO_COUNT; channel++) {
    pwm_.setPWM(channel, 0, 0);
  }
  // WE NO LONGER KNOW WHERE THE ARMS ARE.
  //
  // Cutting PWM releases the servos; they hold nothing and may settle wherever
  // load takes them. Leaving xAxisFlipped_/yAxisFlipped_ asserting a position
  // makes the next flip a no-op -- the guard in setXAxisFlipped sees the request
  // already satisfied and returns success without moving. Observed as "the Y
  // flippers never came up during grab, but it retracted anyway" (2026-08-14),
  // after an E-stop had cut PWM mid-sequence.
  flipStateKnown_ = false;
}

bool MotorController::setXAxisFlipped(bool flipped,
                                      MotorAbortCallback shouldAbort) {
  // Skip only when the cache is BOTH matching and still trustworthy.
  if (flipStateKnown_ && xAxisFlipped_ == flipped) {
    return true;
  }

  if (!moveServoPairSlow(X_SERVO_1_CHANNEL, X_SERVO_2_CHANNEL,
                         flipped ? SERVO_FLIPPED_ANGLE : SERVO_HOME_ANGLE,
                         shouldAbort)) {
    return false;
  }

  xAxisFlipped_ = flipped;
  flipStateKnown_ = true;
  return true;
}

bool MotorController::setYAxisFlipped(bool flipped,
                                      MotorAbortCallback shouldAbort) {
  // Skip only when the cache is BOTH matching and still trustworthy.
  if (flipStateKnown_ && yAxisFlipped_ == flipped) {
    return true;
  }

  if (!moveServoPairSlow(Y_SERVO_1_CHANNEL, Y_SERVO_2_CHANNEL,
                         flipped ? SERVO_FLIPPED_ANGLE : SERVO_HOME_ANGLE,
                         shouldAbort)) {
    return false;
  }

  yAxisFlipped_ = flipped;
  flipStateKnown_ = true;
  return true;
}

bool MotorController::isXAxisFlipped() const {
  return xAxisFlipped_;
}

bool MotorController::isYAxisFlipped() const {
  return yAxisFlipped_;
}

bool MotorController::areBothAxesFlipped() const {
  return xAxisFlipped_ && yAxisFlipped_;
}

bool MotorController::moveServoPairSlow(uint8_t servo1, uint8_t servo2,
                                        int targetAngle,
                                        MotorAbortCallback shouldAbort) {
  int currentAngle = servoAngle_[servo1];

  if (shouldAbort != nullptr && shouldAbort()) {
    stopAll();
    return false;
  }

  if (targetAngle > currentAngle) {
    for (int angle = currentAngle; angle <= targetAngle; angle++) {
      if (shouldAbort != nullptr && shouldAbort()) {
        stopAll();
        return false;
      }

      setServoAngle(servo1, angle);
      setServoAngle(servo2, angle);
      servoAngle_[servo1] = angle;
      servoAngle_[servo2] = angle;
      delay(SERVO_STEP_DELAY_MS);
    }
  } else if (targetAngle < currentAngle) {
    for (int angle = currentAngle; angle >= targetAngle; angle--) {
      if (shouldAbort != nullptr && shouldAbort()) {
        stopAll();
        return false;
      }

      setServoAngle(servo1, angle);
      setServoAngle(servo2, angle);
      servoAngle_[servo1] = angle;
      servoAngle_[servo2] = angle;
      delay(SERVO_STEP_DELAY_MS);
    }
  }

  return true;
}

void MotorController::setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);

  int pulseUs = map(angle, 0, 180, SERVO_MIN_US, SERVO_MAX_US);
  setServoPulse(channel, pulseUs);
}

void MotorController::setServoPulse(uint8_t channel, int pulseUs) {
  int pulseLengthUs = 1000000 / SERVO_FREQ_HZ;
  int ticks = (pulseUs * 4096) / pulseLengthUs;

  pwm_.setPWM(channel, 0, ticks);
}
