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

  // Zero means "not configured separately" for all three, so a default-
  // constructed CornerConfig behaves exactly as it did before hysteresis and
  // asymmetric debounce existed.
  const uint16_t a_min = cfg_.assert_min_mm ? cfg_.assert_min_mm : cfg_.band_min_mm;
  const uint16_t a_max = cfg_.assert_max_mm ? cfg_.assert_max_mm : cfg_.band_max_mm;
  const uint8_t rel_deb = cfg_.release_debounce ? cfg_.release_debounce : cfg_.debounce;

  for (size_t i = 0; i < n; ++i) {
    const ZoneReading& z = frame.zones[i];
    mm_[i] = z.mm;

    // THE TEST DEPENDS ON THE CURRENT STATE -- that is the hysteresis. An absent
    // corner must reach the narrow assert band to come on; a present one is only
    // dropped once it leaves the wider release band. A reading parked between
    // the two boundaries therefore holds whatever it already was, instead of
    // flickering with millimetre-scale noise.
    const bool raw = present_[i]
                         ? (z.valid && z.mm >= cfg_.band_min_mm && z.mm <= cfg_.band_max_mm)
                         : (z.valid && z.mm >= a_min && z.mm <= a_max);

    if (raw == present_[i]) {
      disagree_[i] = 0;  // steady — reset the flip counter
    } else if (++disagree_[i] >= (present_[i] ? rel_deb : cfg_.debounce)) {
      present_[i] = raw;  // enough disagreeing samples — flip
      disagree_[i] = 0;
    }
  }
}

} // namespace tb
