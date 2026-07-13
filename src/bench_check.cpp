// Standalone bench check + auto edge-align — NOT the docking firmware.
// Rig: 2x VL53L0X (front-left = A, front-right = B, facing up) + 4 mecanum
// wheels on 2x L298N,
//      PWM enables on GPIO4/5 (board A) and GPIO6/7 (board B).
//
//   pio run -e bench -t upload
//   python teleop.py            # live WASD control (pio monitor is broken on
//   this host)
//
// MANUAL jog:
//   w/a/s/d  fwd/left/back/right   q/e rotate CCW/CW   space stop
//   + / -    jog speed             1-4 pulse-hold one wheel (wiring check)   ?
//   help
// AUTO-ALIGN:
//   fires automatically when a front sensor first sees the board (edge sensed),
//   or press g to start it manually;  x aborts back to MANUAL;  t toggles
//   auto-trigger. ORIENT: rotate toward the lagging sensor until BOTH see the
//   board, then stop -> ALIGNED.
#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

// ---- Pins -------------------------------------------------------------------
static const uint8_t PIN_SDA = 38, PIN_SCL = 39;
static const uint8_t PIN_XSHUT_A = 40, PIN_XSHUT_B = 41;
static const uint8_t MUX_ADDR = 0x70;

struct MotorPins {
  uint8_t en, in1, in2;
};
static const MotorPins MOTOR[4] = {
    {4, 15, 16}, // FL  (L298N board A)
    {5, 17, 18}, // FR  (L298N board A)
    {7, 10, 11}, // RL  (board B ch2 — swapped: rear wheels were cross-wired)
    {6, 8, 9},   // RR  (board B ch1)
};
static const char *WHEEL[4] = {"FL", "FR", "RL", "RR"};

// ---- Alignment tunables (bench-tune to the real mounting height) ------------
static const uint16_t BAND_MIN_MM =
    20; // board "present" when reading in [min, max]
static const uint16_t BAND_MAX_MM = 500;
static const uint8_t DEBOUNCE_N =
    2; // consecutive disagreeing samples to flip present
static const int ALIGN_ROT_DUTY = 180; // rotate duty for ORIENT (0..255)
static const int CENTER_DUTY =
    180; // slow creep once UNDER the board — visibly slower
static const float CREEP_NOMINAL_MMPS =
    150.0f; // odometry scale (cancels at the midpoint)
static const float CENTER_TOL_MM =
    12.0f; // stop when within this of the midpoint
static const float FRONT_OFFSET_MM =
    0.0f; // sensor->platform-centre offset (0 = centre the sensors)
static const uint32_t ORIENT_TIMEOUT_MS = 10000;
static const uint32_t LOST_TIMEOUT_MS =
    1500; // both sensors lost this long -> abort
static const uint32_t CENTER_TIMEOUT_MS = 12000; // per centring phase

// ---- ToF --------------------------------------------------------------------
static VL53L0X tofA, tofB;
static bool okA = false, okB = false;
static bool useMux = false;

static void muxSelect(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(static_cast<uint8_t>(1u << ch));
  Wire.endTransmission();
}
static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}
static void i2cScan() {
  Serial.println("# I2C scan:");
  uint8_t n = 0;
  for (uint8_t a = 1; a < 127; ++a)
    if (i2cPresent(a)) {
      Serial.printf("#   found 0x%02X\n", a);
      ++n;
    }
  if (!n)
    Serial.println(
        "#   nothing found — check power / SDA-SCL / pull-ups / GND");
}
static bool initOne(VL53L0X &s, const char *tag) {
  s.setTimeout(500);
  bool ok = false;
  for (int i = 0; i < 3 && !ok; ++i)
    ok = s.init();
  if (ok) {
    s.setMeasurementTimingBudget(33000); // 33 ms -> responsive
    s.startContinuous();
    Serial.printf("#   %s: init OK\n", tag);
  } else {
    Serial.printf("#   %s: init FAILED\n", tag);
  }
  return ok;
}
static void bringUpToF() {
  useMux = i2cPresent(MUX_ADDR);
  if (useMux) {
    Serial.println("# mux @0x70 -> A=ch0, B=ch1");
    muxSelect(0);
    okA = initOne(tofA, "A(ch0)");
    muxSelect(1);
    okB = initOne(tofB, "B(ch1)");
    return;
  }
  Serial.println("# no mux -> XSHUT bring-up (A@0x30, B@0x29)");
  pinMode(PIN_XSHUT_A, OUTPUT);
  pinMode(PIN_XSHUT_B, OUTPUT);
  digitalWrite(PIN_XSHUT_A, LOW);
  digitalWrite(PIN_XSHUT_B, LOW);
  delay(20);
  digitalWrite(PIN_XSHUT_A, HIGH);
  delay(20);
  okA = initOne(tofA, "A");
  if (okA)
    tofA.setAddress(0x30);
  digitalWrite(PIN_XSHUT_B, HIGH);
  delay(20);
  okB = initOne(tofB, "B");
}
// Returns range in mm; sets *valid. 0xFFFF distinguishes a dead sensor for
// display.
static uint16_t readOne(VL53L0X &s, bool ok, uint8_t muxCh, bool *valid) {
  if (!ok) {
    *valid = false;
    return 0xFFFF;
  }
  if (useMux)
    muxSelect(muxCh);
  const uint16_t mm = s.readRangeContinuousMillimeters();
  // Reject no-target / signal-fail glitches at the source: only device range
  // status 11 is a valid measurement. Without this the sensor's spurious short
  // reads (e.g. a stray ~159 mm on open air) pass the band test and false-fire
  // the auto-align. 0x14 = RESULT_RANGE_STATUS; status is bits [6:3].
  const uint8_t rangeStatus = (s.readReg(0x14) & 0x78) >> 3;
  *valid = !s.timeoutOccurred() && rangeStatus == 11 && mm < 8000;
  return mm;
}

// Board-present with debounce (mirrors CornerEdgeDetector).
struct Presence {
  bool present = false;
  uint8_t disagree = 0;
  void update(uint16_t mm, bool valid) {
    const bool raw = valid && mm >= BAND_MIN_MM && mm <= BAND_MAX_MM;
    if (raw == present) {
      disagree = 0;
    } else if (++disagree >= DEBOUNCE_N) {
      present = raw;
      disagree = 0;
    }
  }
};
static Presence presA, presB;

// ---- Motors -----------------------------------------------------------------
static int g_speed =
    180; // manual jog duty 0..255 (kept high so the auto slow-down is visible)

// Per-wheel direction calibration. A wheel is "correct" when a positive command
// rolls its top toward the FRONT of the robot; flip any that run backward.
// Bench-tune with 1-4 (spin) + f (flip) + p (print), then bake the mask below.
static bool g_invert[4] = {true, true, false,
                           false}; // FL, FR reversed on hardware; RL, RR normal
static int g_sel = -1;             // wheel currently selected for calibration

static void wheel(uint8_t i, int signedDuty) {
  const MotorPins &m = MOTOR[i];
  const int d = g_invert[i] ? -signedDuty : signedDuty;
  if (d > 0) {
    digitalWrite(m.in1, HIGH);
    digitalWrite(m.in2, LOW);
  } else if (d < 0) {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, HIGH);
  } else {
    digitalWrite(m.in1, LOW);
    digitalWrite(m.in2, LOW);
  }
  analogWrite(m.en, abs(d));
}
static void stopAll() {
  for (uint8_t i = 0; i < 4; ++i)
    wheel(i, 0);
}

// Mecanum mix at duty `spd`, O-configuration rollers (strafe vy + yaw w flipped
// vs. the classic X mix, verified on hardware: forward was correct but strafe &
// rotate were both reversed).
//   FL=vx+vy+w  FR=vx-vy-w  RL=vx-vy+w  RR=vx+vy-w   (each clamped to {-1,0,1})
static void driveMix(int vx, int vy, int w, int spd) {
  wheel(0, constrain(vx + vy + w, -1, 1) * spd); // FL
  wheel(1, constrain(vx - vy - w, -1, 1) * spd); // FR
  wheel(2, constrain(vx - vy + w, -1, 1) * spd); // RL
  wheel(3, constrain(vx + vy - w, -1, 1) * spd); // RR
}

// ---- Mode / state machine ---------------------------------------------------
enum class Mode { Manual, Orient, CenterSeek, CenterReturn, Centered };
static Mode mode = Mode::Manual;
static uint32_t mode_since = 0;
static uint32_t both_lost_since = 0;
static bool prev_any = false; // for rising-edge auto-trigger
static bool armed = false;     // auto-trigger only after a confirmed "no board" baseline
static bool auto_trigger = true;

// Dead-reckon odometry for CENTER_X — integrates the commanded creep velocity.
// Only the ratio matters (we drive to the MIDPOINT at one speed), so the
// absolute scale cancels.
static float odom_x = 0.0f;
static float x_far = 0.0f;
static uint32_t odo_last_ms = 0;
static void odoReset() {
  odom_x = 0.0f;
  odo_last_ms = millis();
}
static void odoStep(int vx_sign) {
  const uint32_t now = millis();
  const float dt = (now - odo_last_ms) / 1000.0f;
  odo_last_ms = now;
  odom_x += vx_sign * CREEP_NOMINAL_MMPS * dt;
}

static const char *modeName() {
  switch (mode) {
  case Mode::Manual:
    return "MANUAL";
  case Mode::Orient:
    return "ORIENT";
  case Mode::CenterSeek:
    return "CENTER_X:seek";
  case Mode::CenterReturn:
    return "CENTER_X:return";
  case Mode::Centered:
    return "CENTERED";
  }
  return "?";
}
static void setMode(Mode m) {
  mode = m;
  mode_since = millis();
  both_lost_since = 0;
}
static void startOrient(const char *why) {
  Serial.printf("> AUTO-ALIGN (%s) -> ORIENT\n", why);
  setMode(Mode::Orient);
}
static void abortToManual(const char *why) {
  Serial.printf("> abort (%s) -> MANUAL\n", why);
  stopAll();
  setMode(Mode::Manual);
}

static void help() {
  Serial.println("# MANUAL: w/a/s/d move  q/e rotate  space stop  +/- speed");
  Serial.println("# CAL   : 1-4 spin a wheel fwd  f flip its direction  p "
                 "print invert mask");
  Serial.println("# AUTO  : edge -> ORIENT -> CENTER_X (auto).  g start  x "
                 "abort  t toggle  ? help");
}

static void handleSerial() {
  while (Serial.available()) {
    const char c = Serial.read();
    // Any manual motion key takes control back from auto-align.
    switch (c) {
    case 'w':
      setMode(Mode::Manual);
      driveMix(1, 0, 0, g_speed);
      Serial.println("> fwd");
      break;
    case 's':
      setMode(Mode::Manual);
      driveMix(-1, 0, 0, g_speed);
      Serial.println("> back");
      break;
    case 'a':
      setMode(Mode::Manual);
      driveMix(0, -1, 0, g_speed);
      Serial.println("> left");
      break;
    case 'd':
      setMode(Mode::Manual);
      driveMix(0, 1, 0, g_speed);
      Serial.println("> right");
      break;
    case 'q':
      setMode(Mode::Manual);
      driveMix(0, 0, -1, g_speed);
      Serial.println("> rot CCW");
      break;
    case 'e':
      setMode(Mode::Manual);
      driveMix(0, 0, 1, g_speed);
      Serial.println("> rot CW");
      break;
    case ' ':
      setMode(Mode::Manual);
      stopAll();
      Serial.println("> stop");
      break;
    case 'x':
      abortToManual("user");
      break;
    case 'g':
      if (mode == Mode::Manual)
        startOrient("manual g");
      break;
    case 't':
      auto_trigger = !auto_trigger;
      Serial.printf("> auto-trigger %s\n", auto_trigger ? "ON" : "OFF");
      break;
    case '+':
      g_speed = min(255, g_speed + 20);
      Serial.printf("> speed %d\n", g_speed);
      break;
    case '-':
      g_speed = max(0, g_speed - 20);
      Serial.printf("> speed %d\n", g_speed);
      break;
    case '1':
    case '2':
    case '3':
    case '4': {
      const uint8_t i = c - '1';
      g_sel = i;
      setMode(Mode::Manual);
      stopAll();
      wheel(i, g_speed); // spin "forward" (invert applied) — watch the top of
                         // the wheel
      Serial.printf("> CAL %s forward%s — top should roll toward FRONT;  "
                    "f=flip  space=stop\n",
                    WHEEL[i], g_invert[i] ? " [inverted]" : "");
      break;
    }
    case 'f':
      if (g_sel >= 0) {
        g_invert[g_sel] = !g_invert[g_sel];
        wheel(g_sel,
              g_speed); // re-spin so you can verify the corrected direction
        Serial.printf("> %s now %s — re-check FRONT\n", WHEEL[g_sel],
                      g_invert[g_sel] ? "INVERTED" : "normal");
      } else {
        Serial.println("> pick a wheel first (1-4)");
      }
      break;
    case 'p':
      Serial.printf("> invert mask {FL,FR,RL,RR} = {%d, %d, %d, %d}\n",
                    g_invert[0], g_invert[1], g_invert[2], g_invert[3]);
      break;
    case '?':
      help();
      break;
    default:
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println(
      "\n# TrolleyBot bench + auto edge-align (2 ToF, 4 mecanum/L298N)");

  for (uint8_t i = 0; i < 4; ++i) {
    pinMode(MOTOR[i].in1, OUTPUT);
    pinMode(MOTOR[i].in2, OUTPUT);
    pinMode(MOTOR[i].en, OUTPUT);
  }
  stopAll();

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  i2cScan();
  bringUpToF();
  help();
}

void loop() {
  handleSerial();

  // Fresh sensor read every loop for the state machine.
  bool vA = false, vB = false;
  const uint16_t mmA = readOne(tofA, okA, 0, &vA);
  const uint16_t mmB = readOne(tofB, okB, 1, &vB);
  presA.update(mmA, vA);
  presB.update(mmB, vB);
  const bool a = presA.present, b = presB.present, any = a || b;

  // Auto-trigger on the rising edge of "a front sensor sees the board" while
  // driving manually — but only after we've first confirmed a clear "no board"
  // baseline, so a boot-time / no-target glitch reading can't fire ORIENT.
  if (!any)
    armed = true; // saw genuine clear space -> arm the trigger
  if (mode == Mode::Manual && auto_trigger && armed && any && !prev_any)
    startOrient("edge sensed");
  prev_any = any;

  switch (mode) {
  case Mode::Manual:
    // Motion is set-and-hold from handleSerial(); nothing to do here.
    break;

  case Mode::Orient:
    if (a && b) {
      stopAll();
      Serial.println("> ORIENT done: squared up on the near edge -> CENTER_X "
                     "(slow creep)");
      odoReset();
      setMode(Mode::CenterSeek);
    } else {
      // Rotate toward the lagging sensor. Sign convention: verify on hardware,
      // flip if wrong.
      const int w = (a && !b) ? -1 : +1;
      driveMix(0, 0, w, ALIGN_ROT_DUTY);
      if (!a && !b) {
        if (both_lost_since == 0)
          both_lost_since = millis();
        else if (millis() - both_lost_since > LOST_TIMEOUT_MS)
          abortToManual("board lost");
      } else {
        both_lost_since = 0;
      }
      if (millis() - mode_since > ORIENT_TIMEOUT_MS)
        abortToManual("orient timeout");
    }
    break;

  case Mode::CenterSeek:
    // Creep forward UNDER the board (visibly slow) until BOTH sensors clear the
    // far edge.
    driveMix(1, 0, 0, CENTER_DUTY);
    odoStep(+1);
    if (!a && !b) {
      x_far = odom_x;
      Serial.printf("> far edge reached (x=%.0f) -> return to midpoint %.0f\n",
                    x_far, 0.5f * x_far - FRONT_OFFSET_MM);
      setMode(Mode::CenterReturn);
    } else if (millis() - mode_since > CENTER_TIMEOUT_MS) {
      abortToManual("centerX seek timeout");
    }
    break;

  case Mode::CenterReturn: {
    // Dead-reckon back to the midpoint between the near and far edges (scale
    // cancels).
    const float target = 0.5f * x_far - FRONT_OFFSET_MM;
    const float err = target - odom_x;
    if (fabsf(err) <= CENTER_TOL_MM) {
      stopAll();
      Serial.printf("> CENTER_X done (x=%.0f, target=%.0f) -> CENTERED\n",
                    odom_x, target);
      setMode(Mode::Centered);
    } else {
      const int dir = (err > 0) ? +1 : -1;
      driveMix(dir, 0, 0, CENTER_DUTY);
      odoStep(dir);
      if (millis() - mode_since > CENTER_TIMEOUT_MS)
        abortToManual("centerX return timeout");
    }
    break;
  }

  case Mode::Centered:
    stopAll(); // hold; any WASD key returns to MANUAL
    break;
  }

  // Sensor edge EVENTS print immediately (these are what you watch during
  // align); a slower heartbeat carries the raw numbers so it doesn't drown the
  // events.
  static bool pa = false, pb = false;
  if (a != pa) {
    Serial.printf("> A %s board\n", a ? "ON" : "off");
    pa = a;
  }
  if (b != pb) {
    Serial.printf("> B %s board\n", b ? "ON" : "off");
    pb = b;
  }

  static uint32_t last = 0;
  if (millis() - last >= 400) {
    last = millis();
    auto fmt = [](uint16_t mm, bool valid) -> String {
      if (mm == 0xFFFF)
        return String("no-init");
      if (!valid)
        return String("oor");
      return String(mm);
    };
    Serial.printf("[%s] A=%s%c B=%s%c  x=%.0f\n", modeName(),
                  fmt(mmA, vA).c_str(), a ? '*' : ' ', fmt(mmB, vB).c_str(),
                  b ? '*' : ' ', odom_x);
  }
}
