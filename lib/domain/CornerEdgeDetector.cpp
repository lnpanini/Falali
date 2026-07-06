#include "CornerEdgeDetector.h"

namespace tb {

void CornerEdgeDetector::reset() {
  for (size_t i = 0; i < kNumCorners; ++i) {
    present_[i] = false;
    disagree_[i] = 0;
    mm_[i] = 0;
  }
}

void CornerEdgeDetector::update(const AlignmentFrame& frame) {
  size_t n = frame.zone_count;
  if (n > kNumCorners) n = kNumCorners;

  for (size_t i = 0; i < n; ++i) {
    const ZoneReading& z = frame.zones[i];
    mm_[i] = z.mm;
    const bool raw = z.valid && z.mm >= cfg_.band_min_mm && z.mm <= cfg_.band_max_mm;

    if (raw == present_[i]) {
      disagree_[i] = 0;  // steady — reset the flip counter
    } else if (++disagree_[i] >= cfg_.debounce) {
      present_[i] = raw;  // enough disagreeing samples — flip
      disagree_[i] = 0;
    }
  }
}

} // namespace tb
