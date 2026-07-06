// Shared value types for the TrolleyBot core.
// Pure C++17 — no Arduino, no dynamic allocation (embedded-friendly).
#pragma once

#include <cstddef>
#include <cstdint>

namespace tb {

// Four corner ToF sensors, one per platform corner.
constexpr size_t kNumCorners = 4;
enum class Corner : uint8_t { FL = 0, FR = 1, RL = 2, RR = 3 };

// Maximum zones an AlignmentFrame can carry (>= kNumCorners).
constexpr size_t kMaxZones = 8;

// One time-of-flight zone reading.
struct ZoneReading {
  uint16_t mm = 0;     // measured distance in millimetres (meaningful only if valid)
  bool valid = false;  // false = out-of-range / no return / sensor error
};

// One snapshot from the alignment sensor: N zone readings plus a capture time.
struct AlignmentFrame {
  ZoneReading zones[kMaxZones];
  size_t zone_count = 0;
  uint32_t t_ms = 0;
};

// Planar pose estimate from odometry (body starts at origin, +x forward, +y left).
struct Pose2D {
  float x_mm = 0.0f;
  float y_mm = 0.0f;
  float theta_rad = 0.0f;
};

// Drive command in normalised body-frame units, each in [-1, 1].
struct DriveCommand {
  float vx = 0.0f;     // forward (+) / reverse (-)
  float vy = 0.0f;     // strafe left (+) / right (-)   [mecanum]
  float omega = 0.0f;  // yaw CCW (+) / CW (-)
};

// Aggregated hardware fault inputs the safety monitor evaluates.
struct FaultFlags {
  bool motor_alarm = false;        // any BLD120A ALARM asserted (wire-OR'd)
  bool clamp_overcurrent = false;  // BTS7960 current sense above the stall limit
  bool estop = false;              // physical E-stop asserted
};

// High-level operator commands (parsed from serial by the telemetry adapter).
enum class Command : uint8_t { None, Dock, Abort, Unclamp, Status };

// Clamp actuation intent.
enum class ClampAction : uint8_t { Stop, Open, Close };

} // namespace tb
