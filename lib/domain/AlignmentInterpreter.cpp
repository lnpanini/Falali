#include "AlignmentInterpreter.h"

namespace tb {

namespace {
// Count set bits in the low `n` bits of `mask`.
uint8_t popcount_low(uint16_t mask, uint8_t n) {
  const uint16_t m = (n >= 16) ? mask : static_cast<uint16_t>(mask & ((1u << n) - 1u));
  uint16_t v = m;
  uint8_t c = 0;
  while (v) {
    c = static_cast<uint8_t>(c + (v & 1u));
    v = static_cast<uint16_t>(v >> 1);
  }
  return c;
}
} // namespace

AlignmentInterpreter::AlignmentInterpreter(const AlignmentConfig& cfg, const IClock& clock)
    : cfg_(cfg), clock_(clock) {}

void AlignmentInterpreter::reset() {
  for (auto& h : hit_hist_) h = 0;
  frames_seen_ = 0;
  last_valid_ms_ = 0;
  ever_valid_ = false;
  stable_ = false;
  stable_since_ms_ = 0;
  state_ = AlignmentState{};
}

void AlignmentInterpreter::update(const AlignmentFrame& frame) {
  const uint32_t now = clock_.millis();
  const uint8_t window =
      (cfg_.window == 0 || cfg_.window > kZoneWindow) ? kZoneWindow : cfg_.window;

  size_t zones = frame.zone_count;
  if (zones > kMaxZones) zones = kMaxZones;

  // Fold this frame into each zone's rolling hit history.
  bool any_valid = false;
  for (size_t i = 0; i < zones; ++i) {
    const ZoneReading& z = frame.zones[i];
    const bool in_band =
        z.valid && z.mm >= cfg_.band_min_mm && z.mm <= cfg_.band_max_mm;
    if (z.valid) any_valid = true;
    hit_hist_[i] = static_cast<uint16_t>((hit_hist_[i] << 1) | (in_band ? 1u : 0u));
  }

  if (any_valid) {
    last_valid_ms_ = now;
    ever_valid_ = true;
  }
  if (frames_seen_ < window) ++frames_seen_;
  const uint8_t denom = frames_seen_ == 0 ? 1 : frames_seen_;

  // under_trolley = mean per-zone in-band hit-rate over the window.
  // Left/right coverage feeds centring and the signed lateral bias.
  float sum_rate = 0.0f, left_sum = 0.0f, right_sum = 0.0f;
  int left_n = 0, right_n = 0;
  for (size_t i = 0; i < zones; ++i) {
    const float rate = static_cast<float>(popcount_low(hit_hist_[i], window)) / denom;
    sum_rate += rate;
    const int8_t side = cfg_.zone_side[i];
    if (side < 0) {
      left_sum += rate;
      ++left_n;
    } else if (side > 0) {
      right_sum += rate;
      ++right_n;
    }
  }
  const float under = zones == 0 ? 0.0f : sum_rate / static_cast<float>(zones);

  float centred, lateral;
  if (left_n > 0 && right_n > 0) {
    const float lr = left_sum / static_cast<float>(left_n);
    const float rr = right_sum / static_cast<float>(right_n);
    lateral = rr - lr;  // right-heavy coverage -> positive
    const float d = lateral < 0 ? -lateral : lateral;
    centred = 1.0f - d;
    if (centred < 0.0f) centred = 0.0f;
  } else {
    // No lateral geometry configured: cannot judge centring; mirror presence.
    lateral = 0.0f;
    centred = under;
  }

  const bool fresh =
      ever_valid_ && (now - last_valid_ms_) <= cfg_.freshness_timeout_ms;

  // clamp_safe requires all preconditions held continuously for the debounce time.
  const bool conditions = fresh && under >= cfg_.under_trolley_threshold &&
                          centred >= cfg_.centred_threshold;
  if (conditions) {
    if (!stable_) {
      stable_ = true;
      stable_since_ms_ = now;
    }
  } else {
    stable_ = false;
  }
  const bool clamp_safe =
      stable_ && (now - stable_since_ms_) >= cfg_.clamp_debounce_ms;

  state_.under_trolley = under;
  state_.centred = centred;
  state_.lateral = lateral;
  state_.fresh = fresh;
  state_.clamp_safe = clamp_safe;
}

} // namespace tb
