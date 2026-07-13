// Xbox BLE gamepad frontend for the mecanum bench rig.
//   Left stick  = translate (proportional)   right stick X = rotate
//   A button    = auto-alignment ON           B button      = auto-alignment OFF (+abort)
// Manual drive + the shared 2-sensor edge-align (ORIENT -> CENTER_X) both run
// through the shared core (../../src/bench_align.h). Fails safe: BLE disconnect
// stops the motors and disables auto-align.
//
//   pio run -d bench_ble -e esp32-s3-devkitc-1 -t upload
//
// Debug goes to Bluepad32's Console (the align core is print-free), so no
// sdkconfig console change is needed.
#include <Arduino.h>
#include <Bluepad32.h>

#include "bench_align.h" // pulls bench_drive.h + bench_mix.h

static ControllerPtr g_ctl = nullptr;
static const float AXIS_MAX = 512.0f; // Bluepad32 sticks ~ -512..511
static const float DEADZONE = 0.12f;  // absorbs stick drift (~-27 at rest)

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
    alignSetAuto(false); // abort any align in progress
    stopAll();           // fail safe
    Console.println("gamepad disconnected -> STOP");
  }
}

void setup() {
  driveMotorsInit();
  stopAll();
  alignSetup(); // bring up the two front ToF sensors
  BP32.setup(&onConnect, &onDisconnect);
  Console.println("bench_ble gamepad + auto-align ready -- A = align ON, B = align OFF");
}

void loop() {
  BP32.update();
  float vx = 0, vy = 0, w = 0;
  bool aBtn = false, bBtn = false;
  if (g_ctl && g_ctl->isConnected() && g_ctl->isGamepad()) {
    vx = -axisNorm(g_ctl->axisY()); // up = forward
    vy = axisNorm(g_ctl->axisX());  // right = strafe right
    w = axisNorm(g_ctl->axisRX());  // right stick right = rotate CW
    aBtn = g_ctl->a();
    bBtn = g_ctl->b();
  }

  // A rising edge -> auto-align ON;  B rising edge -> OFF (+ abort).
  static bool prevA = false, prevB = false;
  if (aBtn && !prevA) {
    alignSetAuto(true);
    Console.println("> auto-align ON");
  }
  if (bBtn && !prevB) {
    alignSetAuto(false);
    Console.println("> auto-align OFF");
  }
  prevA = aBtn;
  prevB = bBtn;

  // One iteration: reads sensors, runs the state machine, drives the motors.
  alignUpdate(vx, vy, w);

  // Status: print immediately on a mode change, plus a slow heartbeat.
  static const char *lastMode = "";
  const char *m = alignModeName();
  if (m != lastMode) { // literals -> pointer compare is fine
    Console.printf("[%s] auto=%s\n", m, alignAutoEnabled() ? "ON" : "OFF");
    lastMode = m;
  }
  static uint32_t last = 0;
  if (millis() - last >= 500) {
    last = millis();
    Console.printf("[%s] L=%u%s R=%u%s x=%.0f\n", alignModeName(), alignMM(0),
                   alignPresent(0) ? "*" : "", alignMM(1),
                   alignPresent(1) ? "*" : "", alignOdomX());
  }
  delay(10); // yield for Bluepad32 / BTstack
}
