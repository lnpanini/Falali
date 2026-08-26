// Arithmetic over a captured SV->RPM sweep. Deliberately free of Arduino and of
// the sweep state machine, so it can be tested on the host against synthetic
// curves whose answers are known in advance.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace fal {

struct CalPoint {
  int   cmd      = 0;      // 0..255 command
  float sv_volts = 0.0f;   // commanded SV
  float rpm      = 0.0f;   // measured motor-shaft RPM, sign as measured
};

struct LinearFit {
  float slope_rpm_per_volt = 0.0f;
  float intercept_rpm      = 0.0f;
  bool  valid              = false;
};

// Least-squares fit of |rpm| against sv_volts. valid=false if n < 2 or the
// commanded voltage never varies.
LinearFit fitLinear(const CalPoint* pts, size_t n);

// First point (in array order) whose |rpm| reaches rpm_floor. -1 if none.
// On an ascending leg this is break-away.
int breakAwayCmd(const CalPoint* pts, size_t n, float rpm_floor);

// Last point (in array order) whose |rpm| is still at/above rpm_floor. -1 if none.
// On a descending leg this is drop-out. Break-away minus drop-out is stiction.
int dropOutCmd(const CalPoint* pts, size_t n, float rpm_floor);

// First command where |rpm| departs the fit by more than tol_pct. -1 if the
// curve stays linear throughout. Points whose |rpm| is below rpm_floor are
// skipped — a stationary motor below break-away is not a saturation knee.
int kneeCmd(const CalPoint* pts, size_t n, const LinearFit& fit, float tol_pct, float rpm_floor);

// motor_counts / (cpr * wheel_revs). Uses |motor_counts| so FR=LOW works.
// Returns 0 when wheel_revs is 0 (caller treats as invalid).
float gearRatio(int32_t motor_counts, float wheel_revs, int32_t cpr);

} // namespace fal
