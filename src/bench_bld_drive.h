#pragma once
#include <Arduino.h>

#include "bench_mix.h"

// BLD-120A drivetrain for the bench rig — the big-motor equivalent of
// bench_drive.h (which is the small L298N rig). ONE source of truth for the
// BLD-120A pin map, the transistor inversion, and the PWM setup. Shared by the
// serial bench firmware and the Bluepad32 gamepad frontend.
//
// BREADBOARD PINS, not the fabricated PCB. include/pins.h holds the
// netlist-derived map for the board; the two are allowed to differ.
//
// SIGN CONVENTION
// ---------------
// Uses bench_mix.h, where vy positive = strafe RIGHT and w positive = rotate CW.
// That is the OPPOSITE of tb::DriveCommand in lib/ports/types.h (vy+ = LEFT).
// Both are self-consistent; the two conventions cancel because the mixers'
// signs also differ. Do not "fix" one without the other.

struct BldPins {
  const char* name;
  uint8_t brk, en, fr, sv;
};

// As measured on the breadboard 2026-08-06.
inline const BldPins BLD[4] = {
    //  name   BRK  EN   F/R  SV
    {  "FL",   13,  12,  11,  10 },
    {  "FR",   21,  47,  48,  38 },
    {  "RL",   18,  17,  16,  15 },
    {  "RR",   39,  40,  41,  42 },
};

// EN/BRK/F-R go through an NPN that pulls the driver input to COM, so a HIGH on
// the GPIO ASSERTS the line. The output latch resets LOW, which means every line
// boots released and the motors boot disabled — the safe state costs nothing.
inline constexpr uint8_t BLD_ASSERT = HIGH, BLD_RELEASE = LOW;
inline constexpr uint8_t BLD_FWD = LOW, BLD_REV = HIGH;

// 2 kHz is inside both driver manuals' PWM ranges (1–10 kHz and 1–3 kHz).
inline constexpr uint32_t BLD_PWM_HZ = 2000;
inline constexpr uint8_t  BLD_PWM_BITS = 12;
inline constexpr uint16_t BLD_PWM_MAX = (1u << BLD_PWM_BITS) - 1u;

// Per-wheel direction calibration, exactly like g_invert[] on the L298N rig.
// CALIBRATED 2026-08-06 with the 'm' identification routine: the left-hand
// motors are mounted mirrored to the right-hand pair, so FL and RL need their
// F/R line flipped for a "forward" command to actually drive forward.
inline bool g_bld_invert[4] = {true , false, true , false};   // FL FR RL RR

inline bool     g_bld_rev[4] = {false, false, false, false};
inline uint16_t g_bld_duty[4] = {0, 0, 0, 0};

inline void bldInit() {
  for (uint8_t i = 0; i < 4; ++i) {
    // Level BEFORE direction: pinMode(OUTPUT) drives whatever is in the latch,
    // and the latch resets to 0 = released = disabled. Writing first removes any
    // window where a line could be asserted during boot.
    digitalWrite(BLD[i].en, BLD_RELEASE);
    digitalWrite(BLD[i].brk, BLD_RELEASE);
    digitalWrite(BLD[i].fr, BLD_FWD);
    pinMode(BLD[i].en, OUTPUT);
    pinMode(BLD[i].brk, OUTPUT);
    pinMode(BLD[i].fr, OUTPUT);
    ledcAttach(BLD[i].sv, BLD_PWM_HZ, BLD_PWM_BITS);
    ledcWrite(BLD[i].sv, 0);
  }
}

// Drive one wheel with a normalised signed command in [-1, 1].
// Zero releases EN (motor coasts) rather than braking — braking every time the
// stick centres would be violent and would fight the driver's own ramp.
inline void bldWheel(uint8_t i, float v) {
  if (g_bld_invert[i]) v = -v;
  v = constrain(v, -1.0f, 1.0f);

  const bool rev = v < 0.0f;
  const float mag = fabsf(v);

  // Reversing under power is hard on the driver — the manual says stop first —
  // so collapse the duty in the same write that flips the direction line.
  if (rev != g_bld_rev[i]) {
    ledcWrite(BLD[i].sv, 0);
    g_bld_duty[i] = 0;
    g_bld_rev[i] = rev;
    digitalWrite(BLD[i].fr, rev ? BLD_REV : BLD_FWD);
  }

  const uint16_t duty = (uint16_t)lroundf(mag * BLD_PWM_MAX);
  digitalWrite(BLD[i].en, duty > 0 ? BLD_ASSERT : BLD_RELEASE);
  digitalWrite(BLD[i].brk, BLD_RELEASE);
  ledcWrite(BLD[i].sv, duty);
  g_bld_duty[i] = duty;
}

inline void bldStopAll() {
  for (uint8_t i = 0; i < 4; ++i) {
    ledcWrite(BLD[i].sv, 0);
    g_bld_duty[i] = 0;
    digitalWrite(BLD[i].en, BLD_RELEASE);
    digitalWrite(BLD[i].brk, BLD_RELEASE);
  }
}

// Hard stop: disable AND assert the brakes (phases shorted, stops fast).
inline void bldBrakeAll() {
  for (uint8_t i = 0; i < 4; ++i) {
    ledcWrite(BLD[i].sv, 0);
    g_bld_duty[i] = 0;
    digitalWrite(BLD[i].en, BLD_RELEASE);
    digitalWrite(BLD[i].brk, BLD_ASSERT);
  }
}

// Mecanum mix at a global limit in [0, 1]. The limit scales all four wheels
// uniformly — scaling one alone would change the direction of travel.
inline void bldDriveMix(float vx, float vy, float w, float limit) {
  float o[4];
  mixWheels(vx, vy, w, o);
  for (uint8_t i = 0; i < 4; ++i) bldWheel(i, o[i] * limit);
}
