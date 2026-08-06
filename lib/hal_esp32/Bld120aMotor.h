// IMotor adapter for one wheel behind a BLD120A BLDC driver.
//
// SV = speed (PWM, externally RC-filtered to analog — see pins::kSvPwmFreqHz);
// F/R = direction. EN/BRK are GANGED across all four wheels, so pass the same
// shared pins to every motor — the redundant writes to a shared line are harmless.
//
// CONTROL LINES ARE OPEN-DRAIN BY DEFAULT
// ---------------------------------------
// Asserting a line pulls it to COM; releasing it goes Hi-Z and lets the driver's
// own internal pull-up decide the level. This matters because it is the only
// configuration that is safe when the ESP is NOT in control: during the ~200 ms
// between power-on and setup(), during a reset, during a crash, and while the
// board is unplugged, every pin is Hi-Z and every driver therefore sees
// EN released = disabled. A push-pull output cannot offer that guarantee.
//
// It also matches the bench rig, which is the configuration actually validated on
// hardware (src/bench_motor.cpp, PUSH_PULL_CTRL 0). Set kPushPullControl = true
// only if the driver inputs turn out to be plain 3.3 V logic with no pull-up.
//
// Polarities carry the CONFIRMED-2026-07-15 bench values: EN asserted LOW,
// BRK asserted LOW, F/R HIGH = forward.
#pragma once

#include <Arduino.h>

#include "IMotor.h"
#include "PwmPin.h"
#include "pins.h"

namespace tb {

// VERIFIED TRUE 2026-08-05: the transistor adapters are built and the full chain
// (Mac -> ESP32-S3 -> adapter -> BLD-120A -> motor) runs. See
// docs/superpowers/plans/2026-08-05-s3-bench-test-handoff.md.
//
// Setting this FALSE while adapters are fitted inverts every safety polarity:
// kAssert becomes LOW, so enable(false) writes HIGH, turns the NPN on, pulls EN
// to COM and ENABLES the drivetrain. Every safeStop() would start the motors.
//
// TRUE  = EN/BRK/F-R go through an N-MOSFET level/isolation stage (the salvage
//         adapter on the wheel PCB). GPIO HIGH turns the MOSFET on, which pulls
//         the driver input down to COM = ASSERTED. Every control line therefore
//         INVERTS relative to driving the driver pin directly.
// FALSE = GPIO wired straight to the driver input (original bench rig).
//
// Getting this wrong swaps "enable" and "disable" on a live drivetrain, so it is
// a single switch rather than four scattered polarity constants.
//
// NOW TRUE: the adapters are fitted and the drivetrain runs through them.
//
// UNRESOLVED CONTRADICTION in the bench data below — worth settling before the
// robot build. The unpowered diode test read OL both ways, implying no opto; a
// later powered test read 2.20 V / 1.886 V across EN->COM, which conducts. Both
// cannot be right. It does not change this constant (the adapters are in and
// work either way) but it does change whether they were ever strictly required.
//
// Bench evidence so far (2026-08-05): diode test EN->COM and SV->COM read OL in
// BOTH directions with the driver unpowered, so there are NO optocouplers on the
// driver inputs — the opto symbols in the manual's control diagram belong to the
// external PLC it expects you to build. Loading EN with 1.469 kOhm while powered
// gave 0.642 V, i.e. an internal pull-up around 10 kOhm and only a few hundred
// microamps of drive current. A 10 kOhm resistor reads OL in diode mode, which is
// consistent. The rail that pull-up returns to is the open question.
constexpr bool kControlViaMosfet = true;

// With a MOSFET the GPIO drives a gate, never the 5 V node -> push-pull is correct
// and open-drain would never turn the MOSFET on (the gate pulldown would win).
// Without a MOSFET, open-drain is the only safe option (see the header comment).
constexpr bool kPushPullControl = kControlViaMosfet;

// Level that ASSERTS a control line, accounting for the MOSFET inversion.
constexpr uint8_t kAssert   = kControlViaMosfet ? HIGH : LOW;
constexpr uint8_t kRelease  = kControlViaMosfet ? LOW  : HIGH;
// F/R: the driver reads HIGH-at-its-pin as forward, so forward = line RELEASED
// when a MOSFET is inverting, and line driven HIGH when wired direct.
constexpr uint8_t kForward  = kControlViaMosfet ? LOW  : HIGH;
constexpr uint8_t kReverse  = kControlViaMosfet ? HIGH : LOW;

class Bld120aMotor : public IMotor {
public:
  // `alarm` may be pins::kNoPin — the BLD-120A used on this robot has no ALM
  // output. Read the comment on pins::kWheelALARM before "fixing" that.
  Bld120aMotor(uint8_t sv, uint8_t sv_channel, uint8_t dir, uint8_t en, uint8_t brk,
               uint8_t alarm)
      : sv_(sv), sv_ch_(sv_channel), dir_(dir), en_(en), brk_(brk), alarm_(alarm) {}

  void begin() {
    // Set the level BEFORE switching the pin to an output. pinMode(OUTPUT) drives
    // whatever is already in the output latch, which resets to 0 — and 0 on an
    // active-low EN means ENABLED. Writing first closes that window; the previous
    // version of this file left it open and was saved only by BRK happening to
    // default to asserted at the same instant.
    digitalWrite(en_, kRelease);   // released -> disabled
    digitalWrite(brk_, kRelease);  // released -> brake off
    digitalWrite(dir_, kForward);

    const uint8_t mode = kPushPullControl ? OUTPUT : OUTPUT_OPEN_DRAIN;
    pinMode(en_, mode);
    pinMode(brk_, mode);
    pinMode(dir_, mode);

    if (alarm_ != pins::kNoPin) pinMode(alarm_, INPUT_PULLUP);

    sv_pwm_.begin(sv_, sv_ch_, pins::kSvPwmFreqHz, pins::kPwmResBits);
    setSpeed(0.0f);
  }

  void setSpeed(float s) override {
    if (s > 1.0f) s = 1.0f;
    else if (s < -1.0f) s = -1.0f;
    const bool reverse = s < 0.0f;
    digitalWrite(dir_, reverse ? kReverse : kForward);
    sv_pwm_.writeFraction(reverse ? -s : s);
  }

  void enable(bool on) override { digitalWrite(en_, on ? kAssert : kRelease); }
  void brake(bool on) override { digitalWrite(brk_, on ? kAssert : kRelease); }

  // Always false on this hardware: there is no ALM terminal to read. Returning a
  // cheerful "no fault" from an unconnected pull-up was the old behaviour, and it
  // was indistinguishable from working protection — at least this way the absence
  // lives in one place with a name on it.
  bool fault() const override {
    if (alarm_ == pins::kNoPin) return false;
    return digitalRead(alarm_) == LOW;
  }

  // Lets callers and telemetry tell "no fault" apart from "cannot detect faults".
  static constexpr bool faultDetectionAvailable() {
    return pins::kWheelALARM != pins::kNoPin;
  }

private:
  uint8_t sv_, sv_ch_, dir_, en_, brk_, alarm_;
  PwmPin sv_pwm_;
};

} // namespace tb
