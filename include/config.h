// Tunable parameters for the firmware. Kept out of the pure domain so the domain
// stays generic; main.cpp uses these to build the domain config structs.
#pragma once

#include <cstdint>

#include "AlignmentInterpreter.h"
#include "DockingStateMachine.h"

namespace cfg {

// --- Loop timing ---
constexpr uint32_t kControlPeriodMs = 20;     // 50 Hz control tick
constexpr uint32_t kTelemetryPeriodMs = 200;  // 5 Hz status publish

// --- Alignment sensor layout (start: 2 single-point ToF, left + right) ---
constexpr uint8_t kNumZones = 2;
constexpr uint8_t kMuxAddr = 0x70;             // TCA9548A
constexpr uint8_t kMuxChannels[kNumZones] = {0, 1};  // channel per zone

// --- Clamp safety ---
constexpr float kClampStallAmps = 4.0f;        // over-current -> fault (calibrate on bench)

// Build the alignment interpreter config for this sensor layout.
inline tb::AlignmentConfig makeAlignConfig() {
  tb::AlignmentConfig c;
  c.band_min_mm = 20;
  c.band_max_mm = 150;
  c.under_trolley_threshold = 0.60f;
  c.centred_threshold = 0.60f;
  c.freshness_timeout_ms = 300;
  c.clamp_debounce_ms = 400;
  c.zone_side[0] = -1;  // zone 0 = left
  c.zone_side[1] = +1;  // zone 1 = right
  return c;
}

// Docking sequence tuning (struct defaults are sane; override here as needed).
inline tb::DockingConfig makeDockConfig() {
  tb::DockingConfig c;
  return c;
}

} // namespace cfg
