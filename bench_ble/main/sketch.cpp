// Xbox BLE gamepad frontend — drives the mecanum bench rig via the shared
// drivetrain (../../src/bench_drive.h). Left stick = translate (proportional),
// right stick X = rotate. Fails safe: on BLE disconnect the motors stop.
//
//   pio run -d bench_ble -e esp32-s3-devkitc-1 -t upload
//
// Uses Bluepad32's Console (not Arduino Serial) for the few debug prints, so no
// sdkconfig console change is needed for this drive-only build.
#include <Arduino.h>
#include <Bluepad32.h>

#include "bench_drive.h"

static ControllerPtr g_ctl = nullptr;
static const float AXIS_MAX = 512.0f; // Bluepad32 sticks ~ -512..511
static const float DEADZONE = 0.12f;  // absorbs stick drift (~-27 at rest)

// Normalize a raw axis to [-1,1] with a deadzone; rescales so motion starts
// right at the deadzone edge instead of jumping.
static float axisNorm(int raw) {
  float v = (float)raw / AXIS_MAX;
  v = constrain(v, -1.0f, 1.0f);
  if (fabsf(v) < DEADZONE)
    return 0.0f;
  return (v - (v > 0 ? DEADZONE : -DEADZONE)) / (1.0f - DEADZONE);
}

static void onConnect(ControllerPtr c) {
  if (!g_ctl) {
    g_ctl = c;
    Console.printf("gamepad connected: %s\n", c->getModelName());
  }
}
static void onDisconnect(ControllerPtr c) {
  if (g_ctl == c) {
    g_ctl = nullptr;
    stopAll(); // fail safe
    Console.println("gamepad disconnected -> STOP");
  }
}

void setup() {
  driveMotorsInit();
  stopAll();
  BP32.setup(&onConnect, &onDisconnect);
  // Persistent pairing: keep bonds across power cycles so the pad auto-reconnects
  // untethered. Pair ONCE in pairing mode; thereafter just tap the Xbox button.
  // If a stale bond ever causes model='Unknown' churn, reflash to clear NVS.
  Console.println("bench_ble gamepad drive ready -- pair the Xbox pad");
}

void loop() {
  BP32.update();
  float vx = 0, vy = 0, w = 0;
  if (g_ctl && g_ctl->isConnected() && g_ctl->isGamepad()) {
    vx = -axisNorm(g_ctl->axisY()); // left stick up = forward
    vy = axisNorm(g_ctl->axisX());  // left stick right = strafe right
    w = axisNorm(g_ctl->axisRX());  // right stick right = rotate CW
  }
  driveMixF(vx, vy, w, g_speed); // zero intent (or no pad) => stop
  delay(10);                     // yield for Bluepad32 / BTstack
}
