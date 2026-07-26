#include "MotorCalAnalysis.h"

namespace tb {

static inline float absf(float v) { return v < 0.0f ? -v : v; }

LinearFit fitLinear(const CalPoint* pts, size_t n) {
  LinearFit f;
  if (!pts || n < 2) return f;

  float sx = 0, sy = 0, sxx = 0, sxy = 0;
  for (size_t i = 0; i < n; i++) {
    const float x = pts[i].sv_volts;
    const float y = absf(pts[i].rpm);
    sx += x; sy += y; sxx += x * x; sxy += x * y;
  }
  const float nn = (float)n;
  const float denom = nn * sxx - sx * sx;
  if (absf(denom) < 1e-9f) return f;   // no spread in x

  f.slope_rpm_per_volt = (nn * sxy - sx * sy) / denom;
  f.intercept_rpm      = (sy - f.slope_rpm_per_volt * sx) / nn;
  f.valid              = true;
  return f;
}

int breakAwayCmd(const CalPoint* pts, size_t n, float rpm_floor) {
  if (!pts) return -1;
  for (size_t i = 0; i < n; i++)
    if (absf(pts[i].rpm) >= rpm_floor) return pts[i].cmd;
  return -1;
}

int dropOutCmd(const CalPoint* pts, size_t n, float rpm_floor) {
  if (!pts) return -1;
  int last = -1;
  for (size_t i = 0; i < n; i++)
    if (absf(pts[i].rpm) >= rpm_floor) last = pts[i].cmd;
  return last;
}

int kneeCmd(const CalPoint* pts, size_t n, const LinearFit& fit, float tol_pct, float rpm_floor) {
  if (!pts || !fit.valid) return -1;
  for (size_t i = 0; i < n; i++) {
    const float measured = absf(pts[i].rpm);
    if (measured < rpm_floor) continue;           // stationary/deadband, not a knee
    const float predicted = fit.slope_rpm_per_volt * pts[i].sv_volts + fit.intercept_rpm;
    if (predicted <= 0.0f) continue;              // below the useful range
    const float err_pct = 100.0f * (predicted - measured) / predicted;
    if (err_pct > tol_pct) return pts[i].cmd;     // fell short of the line
  }
  return -1;
}

float gearRatio(int32_t motor_counts, float wheel_revs, int32_t cpr) {
  if (wheel_revs == 0.0f || cpr == 0) return 0.0f;
  const float counts = (float)(motor_counts < 0 ? -motor_counts : motor_counts);
  return counts / ((float)cpr * (wheel_revs < 0.0f ? -wheel_revs : wheel_revs));
}

} // namespace tb
