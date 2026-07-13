#pragma once
#include <Arduino.h>

#include "bench_mix.h"

// Shared mecanum drivetrain for the TrolleyBot bench rig — ONE source of truth
// for the L298N pin map, per-wheel direction calibration, and the mecanum mix.
// Included by BOTH the serial bench firmware (src/bench_check.cpp) and the
// Bluepad32 gamepad frontend (bench_ble/main/sketch.cpp). Header-only (C++17
// inline); include from exactly one .cpp per build. No sensor / Serial deps.

struct MotorPins {
  uint8_t en, in1, in2;
};

// L298N pin map. FL/FR = board A; RL/RR = board B (RL/RR swapped to undo the
// original rear cross-wiring). Bench-calibrated — do not edit lightly.
inline const MotorPins MOTOR[4] = {
    {4, 15, 16}, // FL  (board A)
    {5, 17, 18}, // FR  (board A)
    {7, 10, 11}, // RL  (board B ch2)
    {6, 8, 9},   // RR  (board B ch1)
};
inline const char *WHEEL[4] = {"FL", "FR", "RL", "RR"};

// Per-wheel direction calibration (FL, FR reversed on hardware; RL, RR normal).
inline bool g_invert[4] = {true, true, false, false};
inline int g_speed = 180; // manual max duty 0..255
inline int g_sel = -1;    // wheel selected for serial calibration

inline void driveMotorsInit() {
  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(MOTOR[i].in1, OUTPUT);
    pinMode(MOTOR[i].in2, OUTPUT);
    pinMode(MOTOR[i].en, OUTPUT);
  }
}

// Drive one wheel with a signed duty (-255..255); applies the invert mask.
inline void wheel(uint8_t i, int signedDuty) {
  const MotorPins &m = MOTOR[i];
  const int d = g_invert[i] ? -signedDuty : signedDuty;
  if (d > 0) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
  } else if (d < 0) {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, HIGH);
  } else {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
  }
  analogWrite(m.en, abs(d));
}

inline void stopAll() {
  for (uint8_t i = 0; i < 4; ++i)
    wheel(i, 0);
}

// Proportional mecanum mix at duty `spd`; normalized so translate+rotate blends
// don't clip. vx,vy,w in [-1,1].
inline void driveMixF(float vx, float vy, float w, int spd) {
  float o[4];
  mixWheels(vx, vy, w, o);
  for (uint8_t i = 0; i < 4; ++i)
    wheel(i, (int)lroundf(o[i] * spd));
}

// Integer wrapper — preserves the existing state-machine call sites unchanged.
inline void driveMix(int vx, int vy, int w, int spd) {
  driveMixF((float)vx, (float)vy, (float)w, spd);
}
