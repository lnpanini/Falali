// Folds a 4-corner ToF frame into debounced per-corner "board present" booleans.
//
// The trolley base is now a solid board, so a corner "sees the board" when its ToF
// reading is valid and within the expected height band. A small N-sample debounce
// rejects single-reading noise without the heavy windowing the old caged-underside
// design needed.
#pragma once

#include "types.h"

namespace tb {

struct CornerConfig {
  uint16_t band_min_mm = 20;   // board sits within [min, max] above the up-facing sensor
  uint16_t band_max_mm = 400;
  uint8_t debounce = 2;        // consecutive disagreeing samples needed to flip a corner
};

class CornerEdgeDetector {
public:
  explicit CornerEdgeDetector(const CornerConfig& cfg) : cfg_(cfg) {}

  void update(const AlignmentFrame& frame);

  bool present(Corner c) const { return present_[static_cast<size_t>(c)]; }
  bool present(size_t i) const { return present_[i]; }
  uint16_t mm(size_t i) const { return mm_[i]; }

  void reset();

private:
  CornerConfig cfg_;
  bool present_[kNumCorners] = {false, false, false, false};
  uint8_t disagree_[kNumCorners] = {0, 0, 0, 0};  // samples counting toward a flip
  uint16_t mm_[kNumCorners] = {0, 0, 0, 0};
};

} // namespace tb
