#include "SerialTelemetry.h"

#include <Arduino.h>
#include <ArduinoJson.h>
#include <SerialCommands.h>

namespace tb {
namespace {

// One pending operator command, set by the SerialCommands callbacks and consumed
// by poll(). There is a single SerialTelemetry instance, so file scope is fine.
Command g_pending = Command::None;

char g_buffer[32];
SerialCommands g_sc(&Serial, g_buffer, sizeof(g_buffer), "\r\n", " ");

void onDock(SerialCommands*) { g_pending = Command::Dock; }
void onAbort(SerialCommands*) { g_pending = Command::Abort; }
void onUnclamp(SerialCommands*) { g_pending = Command::Unclamp; }
void onStatus(SerialCommands*) { g_pending = Command::Status; }
void onUnknown(SerialCommands* sender, const char* cmd) {
  sender->GetSerial()->print(F("# unknown command: "));
  sender->GetSerial()->println(cmd);
}

SerialCommand g_cmd_dock("DOCK", onDock);
SerialCommand g_cmd_abort("ABORT", onAbort);
SerialCommand g_cmd_unclamp("UNCLAMP", onUnclamp);
SerialCommand g_cmd_status("STATUS", onStatus);

} // namespace

void SerialTelemetry::begin() {
  g_sc.SetDefaultHandler(onUnknown);
  g_sc.AddCommand(&g_cmd_dock);
  g_sc.AddCommand(&g_cmd_abort);
  g_sc.AddCommand(&g_cmd_unclamp);
  g_sc.AddCommand(&g_cmd_status);
}

void SerialTelemetry::pump() { g_sc.ReadSerial(); }

Command SerialTelemetry::poll() {
  const Command c = g_pending;
  g_pending = Command::None;
  return c;
}

void SerialTelemetry::publish(const char* state, const bool* corner_present, size_t n_corners,
                              const Pose2D& pose, bool confirmed, const FaultFlags& f) {
  JsonDocument doc;
  doc["state"] = state;
  JsonArray corners = doc["corners"].to<JsonArray>();  // FL, FR, RL, RR
  for (size_t i = 0; i < n_corners; ++i) corners.add(corner_present[i]);
  doc["x_mm"] = pose.x_mm;
  doc["y_mm"] = pose.y_mm;
  doc["theta"] = pose.theta_rad;
  doc["confirmed"] = confirmed;
  doc["motor_alarm"] = f.motor_alarm;
  doc["overcurrent"] = f.clamp_overcurrent;
  doc["estop"] = f.estop;
  serializeJson(doc, Serial);
  Serial.println();
}

void SerialTelemetry::log(const char* msg) {
  Serial.print(F("# "));
  Serial.println(msg);
}

} // namespace tb
