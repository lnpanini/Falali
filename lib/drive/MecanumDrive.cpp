#include "MecanumDrive.h"

namespace tb {

namespace {
float absf(float v) { return v < 0 ? -v : v; }
float maxf(float a, float b) { return a > b ? a : b; }
} // namespace

MecanumDrive::MecanumDrive(IMotor& fl, IMotor& fr, IMotor& rl, IMotor& rr)
    : fl_(fl), fr_(fr), rl_(rl), rr_(rr) {}

void MecanumDrive::move(const DriveCommand& c) {
  // Standard mecanum mixing. Sign conventions to be validated on hardware.
  float wfl = c.vx - c.vy - c.omega;
  float wfr = c.vx + c.vy + c.omega;
  float wrl = c.vx + c.vy - c.omega;
  float wrr = c.vx - c.vy + c.omega;

  // Normalise so the largest-magnitude wheel is at most 1.0.
  const float peak = maxf(maxf(absf(wfl), absf(wfr)), maxf(absf(wrl), absf(wrr)));
  const float m = peak > 1.0f ? (1.0f / peak) : 1.0f;

  fl_.setSpeed(wfl * m);
  fr_.setSpeed(wfr * m);
  rl_.setSpeed(wrl * m);
  rr_.setSpeed(wrr * m);
}

void MecanumDrive::stop() {
  fl_.setSpeed(0.0f);
  fr_.setSpeed(0.0f);
  rl_.setSpeed(0.0f);
  rr_.setSpeed(0.0f);
}

void MecanumDrive::enable(bool on) {
  fl_.enable(on);
  fr_.enable(on);
  rl_.enable(on);
  rr_.enable(on);
}

void MecanumDrive::brake(bool on) {
  fl_.brake(on);
  fr_.brake(on);
  rl_.brake(on);
  rr_.brake(on);
}

bool MecanumDrive::fault() const {
  return fl_.fault() || fr_.fault() || rl_.fault() || rr_.fault();
}

} // namespace tb
