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

void SerialTelemetry::publish(const char* state, const AlignmentState& a,
                              const FaultFlags& f) {
  JsonDocument doc;
  doc["state"] = state;
  doc["under"] = a.under_trolley;
  doc["centred"] = a.centred;
  doc["lateral"] = a.lateral;
  doc["fresh"] = a.fresh;
  doc["clamp_safe"] = a.clamp_safe;
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
