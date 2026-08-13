// Folds a 4-corner ToF frame into debounced per-corner "board present" booleans.
//
// The trolley base is now a solid board, so a corner "sees the board" when its ToF
// reading is valid and within the expected height band. A small N-sample debounce
// rejects single-reading noise without the heavy windowing the old caged-underside
// design needed.
#pragma once

#include "types.h"

namespace tb {

// THREE INDEPENDENT DEFENCES, because a corner can lie in three different ways.
//
//   isolated spike      one bad frame          -> debounce
//   threshold flicker   sitting on a boundary  -> hysteresis (assert vs release band)
//   wrong target        seeing something else  -> a band tight enough to exclude it
//
// Raising `debounce` alone only addresses the first, and it buys that at the
// cost of latency on every edge -- which is the measurement this whole sequence
// is built on. The other two cost nothing.
struct CornerConfig {
  // RELEASE band. While a corner is present it stays present anywhere in here.
  uint16_t band_min_mm = 20;
  uint16_t band_max_mm = 400;

  // ASSERT band, narrower. A corner must read inside THIS to become present, but
  // only leaves when it falls outside the release band above -- a Schmitt
  // trigger. A sensor sitting near a threshold cannot then chatter: crossing in
  // and crossing out happen at different distances.
  //
  // Zero means "same as the release band", which is plain non-hysteretic
  // behaviour and the default, so existing callers are unchanged.
  uint16_t assert_min_mm = 0;
  uint16_t assert_max_mm = 0;

  uint8_t debounce = 2;          // agreeing samples needed to BECOME present

  // Samples needed to STOP being present. Zero means "same as debounce".
  //
  // Separate because the two directions fail differently and cost differently.
  // A false positive sends Approach off on a non-edge; a false negative stamps
  // a far edge that is not there, and the base parks on the midpoint of a span
  // that does not exist. Both are worth paying latency for -- but the latency
  // lands on different marks, so being able to price them separately matters.
  uint8_t release_debounce = 0;
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
