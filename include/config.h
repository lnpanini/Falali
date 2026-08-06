// Tunable parameters for the firmware. Kept out of the pure domain so the domain
// stays generic; main.cpp uses these to build the domain config structs.
#pragma once

#include <cstdint>

#include "CornerEdgeDetector.h"
#include "DeadReckonOdometry.h"
#include "DockingStateMachine.h"
#include "LinkWatchdog.h"

namespace cfg {

// --- Loop timing ---
constexpr uint32_t kControlPeriodMs = 20;     // 50 Hz control tick
constexpr uint32_t kTelemetryPeriodMs = 200;  // 5 Hz status publish

// --- Alignment sensor layout: 4 corner ToF on the TCA9548A mux ---
// Zone index order MUST match Corner: FL=0, FR=1, RL=2, RR=3.
constexpr uint8_t kNumZones = 4;
constexpr uint8_t kMuxAddr = 0x70;                       // TCA9548A
constexpr uint8_t kMuxChannels[kNumZones] = {0, 1, 2, 3};  // FL, FR, RL, RR

// --- Clamp safety ---
constexpr float kClampStallAmps = 4.0f;  // over-current -> fault (calibrate on bench)

// Corner edge detection: the solid board sits within this height band above the
// up-facing sensors. Tune on the bench.
inline tb::CornerConfig makeCornerConfig() {
  tb::CornerConfig c;
  c.band_min_mm = 20;
  c.band_max_mm = 400;
  c.debounce = 2;
  return c;
}

// Dead-reckoning calibration: platform speed / yaw rate at full command.
inline tb::OdometryCal makeOdometryCal() {
  tb::OdometryCal c;
  c.max_lin_mm_s = 300.0f;
  c.max_ang_rad_s = 1.5f;
  return c;
}

// Docking sequence tuning (struct defaults are sane; override here as needed).
inline tb::DockingConfig makeDockConfig() {
  tb::DockingConfig c;
  return c;
}

// Pi 5 control-link watchdog. 100 ms == 5 missed control ticks: long enough to
// ride out ordinary Linux scheduling jitter, short enough that the platform
// travels only ~30 mm at full speed before the brakes go on.
inline tb::LinkConfig makeLinkConfig() {
  tb::LinkConfig c;
  c.timeout_ms = 100;
  return c;
}

} // namespace cfg
