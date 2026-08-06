/*
 * 4-motor bench rig for the ESP32-S3  —  BLD-120A via transistor adapters
 * =======================================================================
 *   pio run -e bench_s3 -t upload && pio device monitor -e bench_s3
 *   (or drive it with ./teleop.py --log — single keypresses, no Enter)
 *
 * BREADBOARD WIRING, not the PCB. The Wheel Drive PCB may be respun; until then
 * the table below is the single source of truth for this rig. include/pins.h
 * remains the netlist-derived map for the fabricated board — the two are allowed
 * to differ, but ONLY this file describes what is on the breadboard.
 *
 * TRANSISTOR INVERSION
 * --------------------
 * EN/BRK/F-R go through an NPN that pulls the driver input down to COM, so
 * GPIO HIGH = asserted. The output latch resets LOW, so every line boots
 * released and the motor boots disabled — the safe state is free.
 *
 * BEFORE POWERING ON (both are silent failures):
 *   1. RV pot fully ANTICLOCKWISE, or external speed control is ignored.
 *   2. P-sv to maximum, per the SYS manual's "cannot adjust the speed" note.
 */
#include <Arduino.h>
#include <Wire.h>
#include "AS5600.h"

// ===========================================================================
//  PIN TABLE — edit here and nowhere else
// ===========================================================================
struct Wheel {
  const char* name;
  uint8_t brk, en, fr, sv;
  int8_t  enc_ch;      // mux channel for this wheel's AS5600, -1 = none yet
};

// *** VERIFY BEFORE DRIVING ALL FOUR ***
// As supplied 2026-08-06. Two rows (FR and RL) had SV and BRK in the opposite
// order to the other two when transcribed; they are entered here exactly as
// given. If a wheel brakes instead of spinning, or judders under a speed
// command, its SV and BRK are swapped — check that wheel first.
constexpr Wheel WHEELS[4] = {
  //  name   BRK  EN   F/R  SV    encoder mux channel
  {  "FL",   13,  12,  11,  10,   0 },
  {  "FR",   21,  47,  48,  38,   1 },
  {  "RL",   18,  17,  16,  15,   2 },
  {  "RR",   39,  40,  41,  42,   3 },
};

constexpr uint8_t PIN_SDA = 6;      // breadboard I2C to the PCA9548A
constexpr uint8_t PIN_SCL = 7;

// ---- drivetrain geometry ---------------------------------------------------
// AS5600 sits on the MOTOR OUTPUT SHAFT, upstream of the gearbox: shaft RPM is
// 15x wheel RPM. The mm/s figure is FREE-RUNNING RIM SPEED, not ground speed —
// mecanum rollers slip by design, so odometry needs a separate empirical
// calibration with all four wheels. Fine for motor characterisation.
constexpr float GEAR_RATIO   = 15.0f;
constexpr float WHEEL_DIA_MM = 150.0f;

// ---- PWM: 2 kHz is inside both manuals' ranges (1-10 kHz and 1-3 kHz) -------
constexpr uint32_t PWM_HZ   = 2000;
constexpr uint8_t  PWM_BITS = 12;
constexpr uint16_t PWM_MAX  = (1u << PWM_BITS) - 1u;

constexpr uint8_t ASSERT_LVL = HIGH, RELEASE_LVL = LOW;   // transistor inverts
constexpr uint8_t FWD_LVL = LOW, REV_LVL = HIGH;

// ---- I2C mux (PCA9548A / TCA9548A — interchangeable) -----------------------
constexpr uint8_t MUX_ADDR = 0x70;
AS5600  as5600;
bool    g_mux_ok = false;
int8_t  g_mux_cur = -2;

// ---- per-wheel state -------------------------------------------------------
struct State { bool en=false, brk=false, rev=false; uint16_t target=0, applied=0;
               bool enc=false; int32_t last=0; uint32_t t=0; float rpm=0; };
State  g_w[4];

// Per-wheel direction calibration. TRUE means this wheel is mounted or wired
// backwards, so "forward" must drive its F/R line the other way. Same idea as
// g_invert[] on the L298N rig. Set with 'v', print with 'p', then paste the
// result back into this table so it survives a reflash.
// CALIBRATED 2026-08-06 via 'm' + 'v'. Left side is mounted mirrored to the
// right, so FL/RL need their F/R line flipped for "forward" to mean forward.
bool   g_invert[4] = { true , false, true , false };   // FL FR RL RR

int    g_sel = 0;          // which wheel the keys act on
bool   g_all = false;      // apply to every wheel at once
uint32_t g_last_tick=0, g_last_status=0;

constexpr uint16_t SLEW_PER_TICK = PWM_MAX / 100;
constexpr uint32_t TICK_MS = 10, STATUS_MS = 500;

// ---- Pi wheel-setpoint protocol -------------------------------------------
//   "W <fl> <fr> <rl> <rr>\n"   each -1000..1000 (signed, sign = direction)
//
// The Pi does the mecanum mixing and sends four wheel setpoints; this sketch is
// dumb I/O, matching the migration plan. Every frame also feeds a watchdog: go
// LINK_TIMEOUT_MS without one and all four wheels are disabled and braked.
//
// Unlike LinkWatchdog in the production firmware this does NOT latch — a fresh
// frame re-arms it. That is deliberate for a bench rig with a human holding the
// gamepad; the robot build must use the latching version.
constexpr uint32_t LINK_TIMEOUT_MS = 300;
// Longest a sweep will wait for one speed point to settle. Generous enough to
// cover the driver's 15 s worst-case ACC/DEC ramp.
constexpr uint32_t SWEEP_MAX_MS = 18000;
char     g_line[80];
uint8_t  g_linelen   = 0;
bool     g_line_mode = false;
uint32_t g_last_frame = 0;
bool     g_link_live  = false;

// ---- helpers ---------------------------------------------------------------
static bool i2cPing(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission()==0; }

static void muxSelect(int8_t ch) {
  if (!g_mux_ok || ch == g_mux_cur) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(ch < 0 ? 0x00 : (uint8_t)(1u << ch));
  Wire.endTransmission();
  g_mux_cur = ch;
}

static void applyWheel(int i) {
  digitalWrite(WHEELS[i].en,  g_w[i].en  ? ASSERT_LVL : RELEASE_LVL);
  digitalWrite(WHEELS[i].brk, g_w[i].brk ? ASSERT_LVL : RELEASE_LVL);
  const bool rev = g_w[i].rev ^ g_invert[i];      // calibration flips the line
  digitalWrite(WHEELS[i].fr,  rev ? REV_LVL : FWD_LVL);
}

static void tick() {
  for (int i = 0; i < 4; i++) {
    const uint16_t want = (g_w[i].en && !g_w[i].brk) ? g_w[i].target : 0;
    uint16_t &a = g_w[i].applied;
    if (a < want)      a = min<uint16_t>(want, a + SLEW_PER_TICK);
    else if (a > want) a = (a > SLEW_PER_TICK) ? (uint16_t)(a - SLEW_PER_TICK) : 0;
    ledcWrite(WHEELS[i].sv, a);
  }
}

static void estopAll() {
  for (int i = 0; i < 4; i++) {
    g_w[i].en=false; g_w[i].brk=true; g_w[i].target=0; g_w[i].applied=0;
    applyWheel(i); ledcWrite(WHEELS[i].sv, 0);
  }
  Serial.println(F("\n*** E-STOP: all wheels disabled, braked, SV = 0 ***"));
}

// Poll every wheel's encoder in turn, hopping mux channels as needed.
static void encoderPoll() {
  for (int i = 0; i < 4; i++) {
    if (!g_w[i].enc) continue;
    muxSelect(WHEELS[i].enc_ch);
    const int32_t pos = as5600.getCumulativePosition();
    const uint32_t now = millis(), dt = now - g_w[i].t;
    if (dt >= 250) {
      g_w[i].rpm = ((pos - g_w[i].last) / 4096.0f) * 60000.0f / dt;
      g_w[i].last = pos; g_w[i].t = now;
    }
  }
}

// Apply one "W" frame. Sign selects direction; magnitude 0..1000 maps to duty.
static void applySetpoints(int v[4]) {
  for (int i = 0; i < 4; i++) {
    const bool rev = v[i] < 0;
    int mag = rev ? -v[i] : v[i];
    if (mag > 1000) mag = 1000;
    // Direction changes under power are hard on the driver, so collapse first.
    if (rev != g_w[i].rev) { g_w[i].target = 0; g_w[i].applied = 0; g_w[i].rev = rev; }
    g_w[i].target = (uint32_t)PWM_MAX * mag / 1000;
    g_w[i].en  = mag > 0;
    g_w[i].brk = false;
    applyWheel(i);
  }
  g_last_frame = millis();
  if (!g_link_live) { g_link_live = true; Serial.println(F("# link up (W frames)")); }
}

static void linkWatchdog(uint32_t now) {
  if (!g_link_live || now - g_last_frame < LINK_TIMEOUT_MS) return;
  g_link_live = false;
  for (int i = 0; i < 4; i++) {
    g_w[i].en = false; g_w[i].brk = true; g_w[i].target = 0; g_w[i].applied = 0;
    applyWheel(i); ledcWrite(WHEELS[i].sv, 0);
  }
  Serial.println(F("# LINK LOST - all wheels disabled and braked"));
}

static const char* blocked(int i) {
  if (g_w[i].brk)        return " BLOCKED:brake('n')";
  if (!g_w[i].en)        return " BLOCKED:disabled('e')";
  if (g_w[i].target==0)  return " BLOCKED:duty0('+')";
  return "";
}

static void status() {
  for (int i = 0; i < 4; i++) {
    const float wr = g_w[i].rpm / GEAR_RATIO;
    Serial.printf("%c%s %s%s%s d%4u(%3u%%)", i==g_sel?'>':' ', WHEELS[i].name,
                  g_w[i].en?"EN":"--", g_w[i].brk?"/BRK":"    ", g_w[i].rev?"R":"F",
                  g_w[i].applied, (unsigned)(100UL*g_w[i].applied/PWM_MAX));
    if (g_w[i].enc) Serial.printf(" %5.0frpm %4.0fmm/s", g_w[i].rpm,
                                  wr*PI*WHEEL_DIA_MM/60.0f);
    Serial.println(blocked(i));
  }
  Serial.println();
}

static void help() {
  Serial.println(F(
    "\n--- 4x BLD-120A bench (ESP32-S3) ------------------------------\n"
    "  1-4     select wheel FL/FR/RL/RR      a  toggle ALL-wheels mode\n"
    "  e / d   enable / disable              f / r  forward / reverse\n"
    "  n / b   brake off / on                + / -  duty +/-1%\n"
    "  0-9     duty 0..90%                   w  duty 100%\n"
    "  k       auto sweep the selected wheel (x aborts)\n"
    "  m       MOTOR ID: drive each wheel in turn (x aborts)\n"
    "  v       flip selected wheel's direction   p  print cal table\n"
    "  i       I2C scan       s  status      ?  help\n"
    "  W <fl> <fr> <rl> <rr>   wheel setpoints -1000..1000 (from the Pi)\n"
    "  x       E-STOP ALL (latches brakes — clear with 'n')\n"
    "TO SPIN:  select 1-4  ->  n  ->  f  ->  e  ->  3\n"
    "---------------------------------------------------------------"));
}

static void forEachTarget(void (*fn)(int)) {
  if (g_all) { for (int i=0;i<4;i++) fn(i); } else fn(g_sel);
}

static void sweep() {
  const int i = g_sel;
  if (!g_w[i].enc) { Serial.println(F("sweep needs this wheel's encoder")); return; }
  if (!g_w[i].en || g_w[i].brk) { Serial.println(F("sweep: n, f, e first")); return; }
  Serial.printf("\n# wheel,%s  invert,%d\n"
                "# duty_pct,duty,sv_volts,shaft_rpm,wheel_rpm,rim_mm_s,settled\n",
                WHEELS[i].name, (int)g_invert[i]);
  for (int pct = 0; pct <= 100; pct += 5) {
    g_w[i].target = (uint32_t)PWM_MAX * pct / 100;
    // Wait for the SPEED TO SETTLE, not a fixed dwell. The driver's ACC/DEC pot
    // ramps anywhere from 0.3 s to 15 s, so a fixed 2.5 s wait would sample a
    // motor still accelerating and quietly under-report every point.
    const uint32_t deadline = millis() + SWEEP_MAX_MS;
    float prev = -1e9f; uint8_t stable = 0;
    while (millis() < deadline && stable < 3) {
      const uint32_t now = millis();
      if (now - g_last_tick >= TICK_MS) { g_last_tick = now; tick(); }
      encoderPoll();
      if (Serial.available() && Serial.read()=='x') { estopAll();
        Serial.println(F("# ABORTED")); return; }
      static uint32_t chk = 0;
      if (now - chk >= 300) {
        chk = now;
        const float d = fabsf(g_w[i].rpm - prev);
        stable = (d < fabsf(g_w[i].rpm) * 0.02f + 5.0f) ? stable + 1 : 0;
        prev = g_w[i].rpm;
      }
    }
    const bool settled = stable >= 3;
    const float wr = g_w[i].rpm / GEAR_RATIO;
    Serial.printf("%d,%u,%.2f,%.0f,%.1f,%.0f,%s\n", pct, g_w[i].applied,
                  3.3f*g_w[i].applied/PWM_MAX, g_w[i].rpm, wr,
                  wr*PI*WHEEL_DIA_MM/60.0f, settled ? "ok" : "NOT-SETTLED");
  }
  g_w[i].target = 0;
  Serial.println(F("# sweep done"));
}

// MOTOR IDENTIFICATION — drives each wheel in turn so you can see which
// physical wheel each table row actually controls, and which way it turns.
// Deliberately gentle (15%, just above the measured 5% break-away).
static void identify() {
  Serial.println(F("\n# ==== MOTOR IDENTIFICATION ===="));
  Serial.println(F("# Wheels OFF THE GROUND. Watch which one moves, and note"));
  Serial.println(F("# whether it drives the robot FORWARD. 'x' aborts."));
  const uint16_t duty = (uint32_t)PWM_MAX * 15 / 100;

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {                 // everything else off
      g_w[j].en=false; g_w[j].brk=false; g_w[j].target=0; g_w[j].applied=0;
      applyWheel(j); ledcWrite(WHEELS[j].sv, 0);
    }
    Serial.printf("\n>>> index %d, table row \"%s\"  (invert=%d)  FORWARD 4s\n",
                  i, WHEELS[i].name, (int)g_invert[i]);
    g_w[i].rev = false; g_w[i].brk = false; g_w[i].en = true; g_w[i].target = duty;
    applyWheel(i);

    const uint32_t until = millis() + 4000;
    while (millis() < until) {
      const uint32_t now = millis();
      if (now - g_last_tick >= TICK_MS) { g_last_tick = now; tick(); }
      encoderPoll();
      if (Serial.available() && Serial.read() == 'x') {
        estopAll(); Serial.println(F("# ABORTED")); return;
      }
    }
    if (g_w[i].enc) {
      const float r = g_w[i].rpm;
      Serial.printf("    encoder %+.0f rpm -> %s\n", r,
                    fabsf(r) < 5 ? "NOT TURNING (check SV/BRK wiring!)"
                                 : (r > 0 ? "encoder reads +ve" : "encoder reads -ve"));
    } else {
      Serial.println(F("    (no encoder on this wheel — judge by eye)"));
    }
  }
  for (int j = 0; j < 4; j++) { g_w[j].en=false; g_w[j].target=0; g_w[j].applied=0;
                                applyWheel(j); ledcWrite(WHEELS[j].sv, 0); }
  Serial.println(F("\n# done. Use 'v' on any wheel that turned the WRONG way,"));
  Serial.println(F("# then 'p' to print the table to paste into the source."));
}

// Print the calibration so it survives a reflash.
static void printCal() {
  Serial.println(F("\n// paste into src/bench_s3_motor.cpp"));
  Serial.printf("bool g_invert[4] = { %s, %s, %s, %s };   // %s %s %s %s\n",
                g_invert[0]?"true ":"false", g_invert[1]?"true ":"false",
                g_invert[2]?"true ":"false", g_invert[3]?"true ":"false",
                WHEELS[0].name, WHEELS[1].name, WHEELS[2].name, WHEELS[3].name);
}

static void i2cScan() {
  Serial.println(F("\n--- I2C scan ---"));
  muxSelect(-1);
  Serial.print(F("main bus:"));
  for (uint8_t a=0x08;a<0x78;a++) if (i2cPing(a)) Serial.printf(" 0x%02X", a);
  Serial.println();
  if (!g_mux_ok) { Serial.println(F("no mux at 0x70")); return; }
  for (int8_t ch=0; ch<8; ch++) {
    muxSelect(ch); Serial.printf("  ch%d:", ch); bool any=false;
    for (uint8_t a=0x08;a<0x78;a++)
      if (a!=MUX_ADDR && i2cPing(a)) { Serial.printf(" 0x%02X", a); any=true; }
    Serial.println(any?"":" -");
  }
}

static void encodersBegin() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);
  g_mux_ok = i2cPing(MUX_ADDR);
  Serial.printf("I2C mux 0x%02X: %s\n", MUX_ADDR, g_mux_ok ? "found" : "ABSENT");
  for (int i = 0; i < 4; i++) {
    if (WHEELS[i].enc_ch < 0) continue;
    muxSelect(WHEELS[i].enc_ch);
    if (!i2cPing(0x36)) { Serial.printf("  %s ch%d: no AS5600\n",
                                        WHEELS[i].name, WHEELS[i].enc_ch); continue; }
    as5600.begin();
    g_w[i].enc = as5600.isConnected();
    // A mispositioned magnet returns plausible garbage rather than failing, so
    // AGC is the number that decides whether a calibration is worth anything.
    Serial.printf("  %s ch%d: AS5600 AGC=%3u %s%s%s\n", WHEELS[i].name,
      WHEELS[i].enc_ch, as5600.readAGC(),
      as5600.magnetDetected()?"ok":"*** NO MAGNET ***",
      as5600.magnetTooWeak()?" *TOO WEAK*":"", as5600.magnetTooStrong()?" *TOO STRONG*":"");
    as5600.resetCumulativePosition(0);
    g_w[i].last = 0; g_w[i].t = millis();
  }
  // AGC is the sensor's own GAIN, so it reads the airgap BACKWARDS: a HIGH
  // value means it is straining to see a distant magnet. Full scale is supply
  // dependent -- 0-255 at 5 V but only 0-128 at 3.3 V, and this breakout is on
  // 3V3, so 128 is the RAIL, not the middle. Aim for ~64.
  // tools/magtune.py renders this live as a bar for positioning by eye.
  Serial.println(F("  (AGC: aim ~64 on a 3V3 breakout; 128 = rail = magnet too far)"));
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 4; i++) {
    // Level first, then direction — the latch resets LOW = released = safe.
    digitalWrite(WHEELS[i].en, RELEASE_LVL);
    digitalWrite(WHEELS[i].brk, RELEASE_LVL);
    digitalWrite(WHEELS[i].fr, FWD_LVL);
    pinMode(WHEELS[i].en, OUTPUT);
    pinMode(WHEELS[i].brk, OUTPUT);
    pinMode(WHEELS[i].fr, OUTPUT);
    applyWheel(i);
    ledcAttach(WHEELS[i].sv, PWM_HZ, PWM_BITS);
    ledcWrite(WHEELS[i].sv, 0);
  }
  encodersBegin();
  delay(300);
  Serial.println(F("\n4-motor bench ready — all wheels disabled, no brake, 0%."));
  help();
}

void loop() {
  while (Serial.available()) {
    const char c = (char)Serial.read();

    // 'W' starts a line-mode setpoint frame; everything else stays single-key.
    if (!g_line_mode && c == 'W') { g_line_mode = true; g_linelen = 0; continue; }
    if (g_line_mode) {
      if (c == '\n' || c == '\r') {
        g_line[g_linelen] = '\0';
        g_line_mode = false;
        int v[4];
        if (sscanf(g_line, "%d %d %d %d", &v[0], &v[1], &v[2], &v[3]) == 4) {
          applySetpoints(v);
        } else {
          Serial.println(F("# bad W frame"));
        }
      } else if (g_linelen < sizeof(g_line) - 1) {
        g_line[g_linelen++] = c;
      }
      continue;
    }

    if (c >= '1' && c <= '4') { g_sel = c - '1';
      Serial.printf("selected %s\n", WHEELS[g_sel].name); }
    else switch (c) {
      case 'a': g_all = !g_all;
                Serial.println(g_all ? F("ALL-WHEELS mode ON") : F("single wheel")); break;
      case 'e': forEachTarget([](int i){ g_w[i].en = true;  }); break;
      case 'd': forEachTarget([](int i){ g_w[i].en = false; }); break;
      case 'b': forEachTarget([](int i){ g_w[i].brk = true; }); break;
      case 'n': forEachTarget([](int i){ g_w[i].brk = false;}); break;
      case 'f': forEachTarget([](int i){ if (g_w[i].rev) { g_w[i].target=0; g_w[i].applied=0; }
                                         g_w[i].rev = false; }); break;
      case 'r': forEachTarget([](int i){ if (!g_w[i].rev){ g_w[i].target=0; g_w[i].applied=0; }
                                         g_w[i].rev = true;  }); break;
      case '+': forEachTarget([](int i){ g_w[i].target =
                  min<uint32_t>(PWM_MAX, g_w[i].target + PWM_MAX/100); }); break;
      case '-': forEachTarget([](int i){ g_w[i].target =
                  (g_w[i].target > PWM_MAX/100) ? g_w[i].target - PWM_MAX/100 : 0; }); break;
      case 'w': forEachTarget([](int i){ g_w[i].target = PWM_MAX; }); break;
      case 'm': identify(); break;
      case 'p': printCal(); break;
      case 'v': g_invert[g_sel] = !g_invert[g_sel];
                Serial.printf("%s invert -> %d\n", WHEELS[g_sel].name,
                              (int)g_invert[g_sel]); break;
      case 'k': sweep(); break;
      case 'i': i2cScan(); break;
      case 's': status(); break;
      case 'x': estopAll(); break;
      case '?': help(); break;
      default: break;
    }
    if (c != '?' && c != 'i') { for (int i=0;i<4;i++) applyWheel(i); status(); }
  }

  const uint32_t now = millis();
  if (now - g_last_tick >= TICK_MS) { g_last_tick = now; tick(); }
  linkWatchdog(now);
  encoderPoll();
  if (now - g_last_status >= STATUS_MS) { g_last_status = now; status(); }
}
