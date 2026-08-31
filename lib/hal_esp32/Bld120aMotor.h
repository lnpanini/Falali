// IMotor adapter for one wheel behind a BLD120A BLDC driver.
//
// SV = speed (native PWM — see pins::kSvPwmFreqHz); F/R = direction.
//
// EN and BRK are PER-WHEEL on the fabricated board (see pins.h), so each motor
// gets its own four pins. An earlier version of this comment said they were
// ganged; that was the pre-PCB design intent, not the board that exists.
//
// CONTROL LINES GO THROUGH TRANSISTOR ADAPTERS, SO THEY ARE PUSH-PULL
// ------------------------------------------------------------------
// The GPIO drives a transistor gate/base, never the driver's own 5 V node. That
// makes push-pull correct — open-drain could never turn the transistor on, since
// the base/gate pulldown would win and every line would sit released forever.
//
// Safety when the ESP is NOT in control (boot, reset, crash, unplugged): the
// GPIO is Hi-Z, the pulldown holds the transistor off, the driver input floats
// to its own idle level, and EN reads as RELEASED = disabled. The adapter
// preserves the fail-safe that open-drain used to provide.
//
// Polarities are whatever makes the hardware behave, and the hardware has been
// watched: all four wheels enable, brake and reverse correctly as written.
#pragma once

#include <Arduino.h>

#include "IMotor.h"
#include "PwmPin.h"
#include "pins.h"

namespace fal {

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
// EVIDENCE: behavioural, which is the strongest kind available here. All four
// motors enable, run, brake and reverse correctly through the adapters with this
// TRUE. Various multimeter readings were taken during bring-up and contradicted
// each other; they have been discarded rather than reconciled, because they were
// not needed to reach a working drivetrain and would only mislead a reader into
// thinking the driver's input model is understood. It is not, and does not need
// to be until someone redesigns the adapter board.
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
    // pinMode FIRST, then the level.
    //
    // This file used to do the opposite, on the theory that pinMode(OUTPUT)
    // drives whatever is already in the latch and so the level had to be set
    // first to close a boot-time window. That is NOT how Arduino-ESP32 3.x
    // behaves: a digitalWrite to a pin that has not been configured yet is
    // rejected outright and logged as
    //     "IO nn is not set as GPIO. Execute digitalMode(nn, OUTPUT) first."
    // (observed on hardware 2026-08-06 during RL bring-up). The pre-writes were
    // silently discarded, so the window was never actually closed.
    //
    // What keeps boot safe is the adapter polarity, not the ordering: the latch
    // resets to 0 = LOW, and with the inverting transistor stage LOW = RELEASED
    // = driver disabled. The safe state IS the power-on default.
    //
    // *** If kControlViaMosfet ever goes back to false, this is no longer true.
    // *** LOW would then mean ASSERTED, and every driver would enable itself
    // *** between reset and the first digitalWrite below. Re-derive boot safety
    // *** before making that change — do not assume this ordering protects you.
    const uint8_t mode = kPushPullControl ? OUTPUT : OUTPUT_OPEN_DRAIN;
    pinMode(en_, mode);
    pinMode(brk_, mode);
    pinMode(dir_, mode);

    digitalWrite(en_, kRelease);   // released -> disabled
    digitalWrite(brk_, kRelease);  // released -> brake off
    digitalWrite(dir_, kForward);

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

} // namespace fal
