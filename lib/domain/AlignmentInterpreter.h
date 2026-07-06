// Turns noisy, intermittent ToF zone-frames into confidence scores.
//
// The trolley underside may be caged / meshed / barred, so single readings are
// unreliable. Every score is computed over a rolling window of recent frames, so
// occasional dropouts (a gap between bars) do not collapse the estimate.
#pragma once

#include "IClock.h"
#include "types.h"

namespace tb {

struct AlignmentConfig {
  // Expected distance band to the underside, in millimetres. A reading is an
  // "in-band hit" only if it is valid AND within [band_min_mm, band_max_mm].
  uint16_t band_min_mm = 20;
  uint16_t band_max_mm = 150;

  // Number of recent frames considered for hit-rate (clamped to kZoneWindow).
  uint8_t window = kZoneWindow;

  float under_trolley_threshold = 0.60f;  // min under_trolley to consider "under"
  float centred_threshold = 0.60f;        // min centred to consider "aligned"

  uint32_t freshness_timeout_ms = 300;    // no valid reading for longer -> not fresh
  uint32_t clamp_debounce_ms = 400;       // conditions must hold this long for clamp_safe

  // Lateral role of each zone for centring: -1 = left, +1 = right, 0 = ignore.
  // Config, not code, so a new sensor layout is a data change.
  int8_t zone_side[kMaxZones] = {0, 0, 0, 0, 0, 0, 0, 0};
};

class AlignmentInterpreter {
public:
  AlignmentInterpreter(const AlignmentConfig& cfg, const IClock& clock);

  // Fold one sensor frame into the estimate.
  void update(const AlignmentFrame& frame);

  AlignmentState state() const { return state_; }

  // Clear all history (e.g. when a docking attempt restarts).
  void reset();

private:
  const AlignmentConfig cfg_;
  const IClock& clock_;

  uint16_t hit_hist_[kMaxZones] = {0};  // per-zone bitmask; bit0 = most recent in-band hit
  uint8_t frames_seen_ = 0;             // capped at the effective window
  uint32_t last_valid_ms_ = 0;          // last time any zone returned a valid reading
  bool ever_valid_ = false;
  bool stable_ = false;                 // clamp-safe preconditions currently continuous?
  uint32_t stable_since_ms_ = 0;
  AlignmentState state_{};
};

} // namespace tb
