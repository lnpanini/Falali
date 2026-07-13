#pragma once
#include <algorithm>
#include <cmath>

// Pure mecanum mix (O-config rollers), Arduino-free so it is host-testable and
// shared by both the serial bench firmware and the Bluepad32 gamepad frontend.
// vx,vy,w in [-1,1]; writes normalized wheel commands in [-1,1] to out[FL,FR,RL,RR].
// Normalize by the max magnitude so blended translate+rotate scales together (no clip).
inline void mixWheels(float vx, float vy, float w, float out[4]) {
  const float fl = vx + vy + w;
  const float fr = vx - vy - w;
  const float rl = vx - vy + w;
  const float rr = vx + vy - w;
  const float m = std::max(1.0f, std::max(std::max(std::fabs(fl), std::fabs(fr)),
                                          std::max(std::fabs(rl), std::fabs(rr))));
  out[0] = fl / m;
  out[1] = fr / m;
  out[2] = rl / m;
  out[3] = rr / m;
}
