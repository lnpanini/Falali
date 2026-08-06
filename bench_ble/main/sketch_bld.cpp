// Xbox BLE gamepad -> BLD-120A mecanum drivetrain (the BIG motors).
// ==================================================================
//   pio run -d bench_ble -e esp32-s3-devkitc-1 -t upload
//   pio device monitor -d bench_ble -e esp32-s3-devkitc-1
//
// The sibling sketch.cpp drives the small L298N bench rig and adds ToF
// auto-align. This one is deliberately minimal: gamepad -> mecanum mix -> four
// BLD-120A drivers via the transistor adapters. No sensors, no state machine —
// the point is to confirm the drivetrain and the directions.
//
// Controls
//   left stick    translate (proportional)
//   right stick X rotate
//   RB (hold)     boost to the full limit
//   B             stop
//   A             cycle the speed limit  0.15 -> 0.25 -> 0.40
//   D-pad         single-wheel test: up=FL right=FR down=RL left=RR
//
// PAIRING (typed into the serial console — see handleConsole below)
//   P  pairing mode ON   — accept new controllers
//   O  pairing mode OFF  — only previously-paired controllers may connect
//   F  FORGET all stored keys, then re-enter pairing mode. Use this when
//      swapping to a different controller; Bluepad32 remembers the old one
//      and will keep reconnecting to it otherwise.
//   B  print Bluetooth status (own address, firmware, what is connected)
//
// FAILS SAFE
//   * gamepad disconnect  -> immediate stop (Bluepad32 callback)
//   * no gamepad at boot  -> motors stay disabled; nothing spins on power-up
//   * B                   -> stop
//   * starts at limit 0.15 — raise it only once every wheel goes the right way
#include <Arduino.h>
#include <Bluepad32.h>

#include "bench_bld_drive.h"
#include "bench_tof4.h"        // pulls bench_i2cmux.h

// Breadboard I2C to the PCA9548A. Channels 0-3 = encoders, 4-7 = ToF.
static constexpr uint8_t PIN_SDA = 6, PIN_SCL = 7;
static bool g_tof_stream = false;   // 'T' toggles a live range dump

static ControllerPtr g_ctl = nullptr;
static bool g_pairing = true;   // Bluepad32 scans by default at setup()
static const float AXIS_MAX = 512.0f;  // Bluepad32 sticks ~ -512..511
static const float DEADZONE = 0.12f;   // from sketch.cpp — absorbs stick drift

// Deliberately conservative. The drivetrain reaches well over 1 m/s at full
// command, which is not a speed to discover a reversed wheel at.
static const float LIMITS[] = {0.15f, 0.25f, 0.40f};
static uint8_t g_limit_idx = 0;

static float axisNorm(int raw) {
  float v = (float)raw / AXIS_MAX;
  v = constrain(v, -1.0f, 1.0f);
  if (fabsf(v) < DEADZONE) return 0.0f;
  // Rescale outside the deadzone so the stick still reaches 1.0 at the stop.
  return (v - (v > 0 ? DEADZONE : -DEADZONE)) / (1.0f - DEADZONE);
}

// Bluepad32 starts scanning at setup() and stores keys on first pair, so normal
// use needs no commands at all. These exist for the awkward cases: swapping
// controllers, or stopping the rig grabbing someone else's pad mid-session.
static int g_cal_sel = 0;   // wheel that 'v' acts on

// MOTOR IDENTIFICATION — drive each wheel in turn so you can see which physical
// wheel each table row controls, and which way it turns. Same routine as the
// serial bench build's 'm'. Gentle: 15%, just above the measured 5% break-away.
// BP32.update() keeps running inside the wait so the pad stays connected.
static void identify() {
  Console.println("\n# ==== MOTOR IDENTIFICATION ====");
  Console.println("# Wheels OFF THE GROUND. Note which wheel moves and whether");
  Console.println("# it would drive the robot FORWARD.  'x' or gamepad B aborts.");
  for (int i = 0; i < 4; ++i) {
    bldStopAll();
    Console.printf("\n>>> row %d \"%s\"  (invert=%d)  FORWARD 4s\n",
                   i, BLD[i].name, (int)g_bld_invert[i]);
    const uint32_t until = millis() + 4000;
    while (millis() < until) {
      BP32.update();
      bldWheel(i, 0.15f);
      if (Serial.available() && (char)Serial.read() == 'x') {
        bldStopAll(); Console.println("# ABORTED"); return;
      }
      if (g_ctl && g_ctl->isConnected() && g_ctl->b()) {
        bldStopAll(); Console.println("# ABORTED (B)"); return;
      }
      delay(10);
    }
  }
  bldStopAll();
  Console.println("\n# done. '1'-'4' select a wheel, 'v' flips one that ran");
  Console.println("# backwards, 'p' prints the table to paste into the source.");
}

static void printCal() {
  Console.println("\n// paste into src/bench_bld_drive.h");
  Console.printf("inline bool g_bld_invert[4] = { %s, %s, %s, %s };  // %s %s %s %s\n",
                 g_bld_invert[0] ? "true " : "false", g_bld_invert[1] ? "true " : "false",
                 g_bld_invert[2] ? "true " : "false", g_bld_invert[3] ? "true " : "false",
                 BLD[0].name, BLD[1].name, BLD[2].name, BLD[3].name);
}

static void printBtStatus() {
  const uint8_t* a = BP32.localBdAddress();
  Console.printf("\n-- bluetooth --------------------------------\n");
  Console.printf("  bluepad32 fw : %s\n", BP32.firmwareVersion());
  Console.printf("  this esp32   : %02X:%02X:%02X:%02X:%02X:%02X\n",
                 a[0], a[1], a[2], a[3], a[4], a[5]);
  if (g_ctl && g_ctl->isConnected()) {
    Console.printf("  connected    : %s  battery %d\n",
                   g_ctl->getModelName().c_str(), g_ctl->battery());
  } else {
    Console.printf("  connected    : (none)\n");
  }
  Console.printf("  pairing mode : %s\n", g_pairing ? "ON" : "off");
  Console.printf("---------------------------------------------\n");
}

// Motors are stopped before any pairing change: putting the radio into
// discovery while a wheel is spinning is not a combination worth having.
static void handleConsole() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c >= '1' && c <= '4') {
      g_cal_sel = c - '1';
      Console.printf("cal wheel -> %s\n", BLD[g_cal_sel].name);
      continue;
    }
    switch (c) {
      case 'P':
        bldStopAll(); g_pairing = true;
        BP32.enableNewBluetoothConnections(true);
        Console.println("pairing ON — hold the Xbox button + the small PAIR button");
        Console.println("until the logo flashes FAST, then wait ~10s");
        break;
      case 'O':
        g_pairing = false;
        BP32.enableNewBluetoothConnections(false);
        Console.println("pairing OFF — only known controllers may connect");
        break;
      case 'F':
        bldStopAll();
        BP32.forgetBluetoothKeys();
        g_pairing = true;
        BP32.enableNewBluetoothConnections(true);
        Console.println("forgot all stored keys; pairing ON");
        Console.println("ALSO remove 'Xbox Wireless Controller' from any phone/PC");
        Console.println("it is paired with, or it will reconnect there instead");
        break;
      case 'm': identify(); break;
      case 'p': printCal(); break;
      case 'v':
        g_bld_invert[g_cal_sel] = !g_bld_invert[g_cal_sel];
        Console.printf("%s invert -> %d\n", BLD[g_cal_sel].name,
                       (int)g_bld_invert[g_cal_sel]);
        break;
      case 't': tof4Read(); tof4Print(Console); break;
      case 'T': g_tof_stream = !g_tof_stream;
                Console.printf("ToF stream %s\n", g_tof_stream ? "ON" : "off"); break;
      case 'i': muxScan(Console); break;
      case 'B': printBtStatus(); break;
      case 'x': bldStopAll(); Console.println("STOP"); break;
      case '?':
        Console.println("bluetooth : P pair on | O pair off | F forget | B status");
        Console.println("calibrate : m identify | 1-4 select | v flip | p print");
        Console.println("sensors   : t read ToF once | T stream | i I2C scan");
        Console.println("            x stop");
        break;
      default: break;
    }
  }
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
    bldStopAll();  // fail safe — do this before anything else
    Console.println("gamepad disconnected -> STOP");
  }
}

void setup() {
  bldInit();
  bldStopAll();
  if (muxBegin(PIN_SDA, PIN_SCL)) {
    Console.println("I2C mux 0x70: found");
    tof4Begin(Console);
  } else {
    Console.println("I2C mux 0x70: ABSENT — no ToF");
  }

  BP32.setup(&onConnect, &onDisconnect);
  BP32.enableVirtualDevice(false);   // no phantom mouse device from the pad
  Console.println("BLD-120A gamepad ready.");
  Console.println("  serial: m identify | v flip dir | p print cal | ? help");
  Console.println("  serial: P pair on | O pair off | F forget keys | B status");
  Console.println("  left stick drive/strafe | right stick rotate");
  Console.println("  A = cycle limit | RB = boost | B = stop");
  Console.println("  D-pad = single wheel: up FL, right FR, down RL, left RR");
  Console.printf("  limit %.2f  --  WHEELS OFF THE GROUND FIRST\n", LIMITS[g_limit_idx]);
}

void loop() {
  handleConsole();
  BP32.update();

  float vx = 0, vy = 0, w = 0;
  bool boost = false, stop = false;
  uint8_t dpad = 0;

  const bool live = g_ctl && g_ctl->isConnected() && g_ctl->isGamepad();
  if (live) {
    vx = -axisNorm(g_ctl->axisY());   // up = forward
    vy = axisNorm(g_ctl->axisX());    // right = strafe right (bench_mix sign)
    w  = axisNorm(g_ctl->axisRX());   // right stick right = rotate CW
    boost = g_ctl->r1();
    stop  = g_ctl->b();
    dpad  = g_ctl->dpad();

    static bool prevA = false;
    const bool a = g_ctl->a();
    if (a && !prevA) {
      g_limit_idx = (g_limit_idx + 1) % (sizeof(LIMITS) / sizeof(LIMITS[0]));
      Console.printf("limit -> %.2f\n", LIMITS[g_limit_idx]);
    }
    prevA = a;
  }

  const float limit = boost ? 1.0f : LIMITS[g_limit_idx];

  if (!live || stop) {
    bldStopAll();
  } else if (dpad) {
    // Single-wheel test — the fastest way to find a reversed or dead wheel.
    // Each direction spins exactly one wheel forward at the current limit.
    int only = -1;
    if (dpad & DPAD_UP)         only = 0;   // FL
    else if (dpad & DPAD_RIGHT) only = 1;   // FR
    else if (dpad & DPAD_DOWN)  only = 2;   // RL
    else if (dpad & DPAD_LEFT)  only = 3;   // RR
    for (int i = 0; i < 4; ++i) bldWheel(i, i == only ? limit : 0.0f);
    static int lastOnly = -2;
    if (only != lastOnly) {
      Console.printf("wheel test: %s\n", only >= 0 ? BLD[only].name : "-");
      lastOnly = only;
    }
  } else {
    bldDriveMix(vx, vy, w, limit);
  }

  // ToF at 20 Hz — faster than the sensors' 20 ms budget would give anyway.
  static uint32_t tof_t = 0;
  if (millis() - tof_t >= 50) { tof_t = millis(); tof4Read(); }

  static uint32_t last = 0;
  if (millis() - last >= 500) {
    last = millis();
    if (g_tof_stream) tof4Print(Console);
    Console.printf("[%s] lim %.2f%s  vx%+.2f vy%+.2f w%+.2f  duty %4u %4u %4u %4u\n",
                   live ? "live" : "NO PAD", limit, boost ? " BOOST" : "",
                   vx, vy, w, g_bld_duty[0], g_bld_duty[1], g_bld_duty[2], g_bld_duty[3]);
  }

  delay(10);  // yield for Bluepad32 / BTstack
}
