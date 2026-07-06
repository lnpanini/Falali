// Shared value types for the TrolleyBot core.
// Pure C++17 — no Arduino, no dynamic allocation (embedded-friendly).
#pragma once

#include <cstddef>
#include <cstdint>

namespace tb {

// Maximum number of alignment zones the interpreter can track.
// Fixed so the pure core stays allocation-free regardless of sensor layout.
constexpr size_t kMaxZones = 8;

// Number of recent frames the alignment interpreter remembers per zone.
// Stored as a bitmask, so this must be <= 16 (bits in a uint16_t).
constexpr uint8_t kZoneWindow = 16;

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

// Confidence-scored interpretation the docking logic consumes.
struct AlignmentState {
  float under_trolley = 0.0f;  // 0..1 confidence the robot is beneath the trolley
  float centred = 0.0f;        // 0..1 confidence it is laterally centred
  float lateral = 0.0f;        // signed -1..1 (right-heavy +) — drives the centring strafe
  bool fresh = false;          // false = sensor data stale (too few valid returns lately)
  bool clamp_safe = false;     // the single derived gate: sustained + fresh + centred + under
};

// Drive command in normalised body-frame units, each in [-1, 1].
struct DriveCommand {
  float vx = 0.0f;     // forward (+) / reverse (-)
  float vy = 0.0f;     // strafe left (+) / right (-)   [mecanum]
  float omega = 0.0f;  // yaw CCW (+) / CW (-)
};

// Aggregated hardware fault inputs the safety monitor evaluates.
struct FaultFlags {
  bool motor_alarm = false;       // any BLD120A ALARM asserted (wire-OR'd)
  bool clamp_overcurrent = false; // BTS7960 current sense above the stall limit
  bool estop = false;             // physical E-stop asserted
};

// High-level operator commands (parsed from serial by the telemetry adapter).
enum class Command : uint8_t { None, Dock, Abort, Unclamp, Status };

// Clamp actuation intent.
enum class ClampAction : uint8_t { Stop, Open, Close };

} // namespace tb
