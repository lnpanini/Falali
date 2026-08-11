// Physical identification for the assembled base.
//
// Answers the question a schematic cannot: which motor output actually drives
// which wheel, which mux channel holds that wheel's encoder, and which ADS1115
// channel carries its current sensor.
//
// It DISCOVERS the mapping rather than assuming it. Drive one motor, see which
// encoder turns and which current channel rises -- that triple is one corner,
// whatever the harness happens to be doing. The FL/FR/RL/RR labels in pins.h are
// the netlist's opinion of the board; this is the robot's.
//
// PINS COME FROM include/pins.h. Nothing is hand-typed.
//
// COMMANDS
//   s  I2C inventory      -- mux, encoders per channel, IMU, ADS1115
//   m  motor mapping      -- spins each wheel briefly, reports enc + mean current
//   t  ToF identify       -- streams all four ranges; cover one at a time
//   x  stop everything
//
// SAFETY: 'm' SPINS ALL FOUR WHEELS, one at a time, 25% duty for 1.5 s.
// WHEELS OFF THE GROUND. Any keypress aborts.
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#include <esp_log.h>

#include "pins.h"

// Adapter polarity, copied from src/bench_pcb_rl.cpp where it is hardware-proven.
// The NPN stage INVERTS: HIGH at the GPIO = asserted at the driver.
static constexpr uint8_t ASSERT_LVL  = HIGH;
static constexpr uint8_t RELEASE_LVL = LOW;
static constexpr uint8_t FWD_LVL     = LOW;   // released = driver sees HIGH = forward
static constexpr uint8_t REV_LVL     = HIGH;

static constexpr uint32_t PWM_HZ   = 2000;    // inside BOTH manuals' ranges
static constexpr uint8_t  PWM_BITS = 12;
static constexpr uint32_t PWM_MAX  = (1u << PWM_BITS) - 1;

static constexpr uint8_t SPIN_DUTY_PCT = 25;   // 10% left the current signal in the noise
static constexpr uint32_t SPIN_MS      = 1500;

static constexpr uint8_t AS5600_ADDR   = 0x36;
static constexpr uint8_t AS5600_RAWANG = 0x0C;   // 12-bit, wraps 0..4095
static constexpr uint8_t AS5600_STATUS = 0x0B;
static constexpr uint8_t AS5600_AGC    = 0x1A;
static constexpr uint8_t ADS1115_ADDR  = 0x48;
static constexpr uint8_t BNO08X_ADDR   = 0x4B;  // ADR strapped HIGH on this breakout (confirmed 2026-08-11)

static const char* CORNER[4] = {"FL", "FR", "RL", "RR"};

static VL53L0X g_tof[4];
static bool    g_tof_ok[4] = {false, false, false, false};

// ---------------------------------------------------------------- I2C helpers

static bool ping(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool muxSelect(uint8_t ch) {
  Wire.beginTransmission(pins::kMuxAddr);
  Wire.write(static_cast<uint8_t>(1u << ch));
  return Wire.endTransmission() == 0;
}

static void muxDisable() {
  Wire.beginTransmission(pins::kMuxAddr);
  Wire.write(static_cast<uint8_t>(0));
  Wire.endTransmission();
}

static bool readReg8(uint8_t addr, uint8_t reg, uint8_t& out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, static_cast<uint8_t>(1)) != 1) return false;
  out = Wire.read();
  return true;
}

static bool readReg16(uint8_t addr, uint8_t reg, uint16_t& out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, static_cast<uint8_t>(2)) != 2) return false;
  out = static_cast<uint16_t>(Wire.read()) << 8;
  out |= Wire.read();
  return true;
}

// ------------------------------------------------------------------ encoders

// Raw angle on mux channel `ch`, or -1 if it did not answer.
static int readAngle(uint8_t ch) {
  if (!muxSelect(ch)) return -1;
  uint16_t raw = 0;
  if (!readReg16(AS5600_ADDR, AS5600_RAWANG, raw)) return -1;
  return static_cast<int>(raw & 0x0FFF);
}

// Shortest signed distance between two 12-bit angles. Without this a wheel
// crossing the 4095->0 boundary reports a 4000-count jump in the wrong
// direction, which would invert the sign the corner mapping depends on.
static int angleDelta(int from, int to) {
  int d = to - from;
  if (d >  2048) d -= 4096;
  if (d < -2048) d += 4096;
  return d;
}

// --------------------------------------------------------------- ADS1115

// Single-shot single-ended read of AIN<ch>. Returns millivolts, or -1.
//
// PGA is +/-4.096 V, NOT +/-2.048 V. hardware-architecture.md quotes 62.5 uV/LSB
// (the 2.048 V setting) but also expects 2.44 V at 30 A inrush -- which that
// range would clip. 4.096 V costs one bit and keeps the inrush visible. The
// ADS1115's absolute input limit is still VDD+0.3, so 3.3 V rails are unaffected.
static long readAdsMv(uint8_t ch) {
  const uint16_t cfg = 0x8000                                   // start single conversion
                     | (static_cast<uint16_t>(4 + ch) << 12)    // MUX: AINch vs GND
                     | (0x1 << 9)                               // PGA +/-4.096 V
                     | (0x1 << 8)                               // single-shot
                     | (0x4 << 5)                               // 128 SPS
                     | 0x0003;                                  // comparator off
  Wire.beginTransmission(ADS1115_ADDR);
  Wire.write(0x01);
  Wire.write(static_cast<uint8_t>(cfg >> 8));
  Wire.write(static_cast<uint8_t>(cfg & 0xFF));
  if (Wire.endTransmission() != 0) return -1;
  delay(10);                                   // 128 SPS -> 7.8 ms
  uint16_t raw = 0;
  if (!readReg16(ADS1115_ADDR, 0x00, raw)) return -1;
  const int16_t signed_raw = static_cast<int16_t>(raw);
  return static_cast<long>(signed_raw) * 125L / 1000L;   // 125 uV/LSB -> mV
}

// -------------------------------------------------------------------- motors

static void motorInit() {
  for (int i = 0; i < 4; ++i) {
    pinMode(pins::kWheelEN[i], OUTPUT);
    pinMode(pins::kWheelBRK[i], OUTPUT);
    pinMode(pins::kWheelFR[i], OUTPUT);
    digitalWrite(pins::kWheelEN[i], RELEASE_LVL);
    digitalWrite(pins::kWheelBRK[i], RELEASE_LVL);
    digitalWrite(pins::kWheelFR[i], FWD_LVL);
    ledcAttach(pins::kWheelSV[i], PWM_HZ, PWM_BITS);
    ledcWrite(pins::kWheelSV[i], 0);
  }
}

static void motorStop(int i) {
  ledcWrite(pins::kWheelSV[i], 0);
  digitalWrite(pins::kWheelEN[i], RELEASE_LVL);
}

static void stopAll() {
  for (int i = 0; i < 4; ++i) motorStop(i);
}

static void motorRun(int i, uint8_t duty_pct, bool reverse) {
  digitalWrite(pins::kWheelFR[i], reverse ? REV_LVL : FWD_LVL);
  digitalWrite(pins::kWheelEN[i], ASSERT_LVL);
  ledcWrite(pins::kWheelSV[i], PWM_MAX * duty_pct / 100u);
}

// ------------------------------------------------------------------ commands

static void inventory() {
  Serial.println(F("\n--- I2C INVENTORY ---"));
  const bool mux = ping(pins::kMuxAddr);
  Serial.printf("  mux    0x%02X : %s\n", pins::kMuxAddr, mux ? "present" : "MISSING");
  Serial.printf("  BNO08x 0x%02X : %s\n", BNO08X_ADDR, ping(BNO08X_ADDR) ? "present" : "MISSING");

  // 0x48-0x4B is shared territory: ADS1115 straps across all four, BNO08x sits
  // at 0x4A or 0x4B. An address alone cannot tell you which part answered, so
  // report what responded and let the count decide. With one of each, two
  // responders is CORRECT, not a collision -- the earlier version of this code
  // cried collision at a perfectly good board.
  Serial.println(F("\n  0x48-0x4B (ADS1115 straps here, BNO08x at 0x4A/0x4B):"));
  uint8_t n_resp = 0;
  for (uint8_t a = 0x48; a <= 0x4B; ++a) {
    if (!ping(a)) continue;
    ++n_resp;
    const char* who = (a <= 0x49) ? "ADS1115 (only it can be here)"
                                  : "ADS1115 or BNO08x -- ambiguous";
    Serial.printf("    0x%02X responds : %s\n", a, who);
  }
  if (n_resp == 0)      Serial.println(F("    nothing responds -- both parts missing"));
  else if (n_resp == 1) Serial.println(F("    only ONE responder: one of the two parts is absent"));
  else if (n_resp == 2) Serial.println(F("    two responders, one of each part -- CORRECT, no conflict"));
  else                  Serial.println(F("    more responders than parts fitted -- investigate"));

  if (!mux) { Serial.println(F("  (no mux -> cannot see encoders)")); return; }
  Serial.println(F("\n  encoders, by mux channel:"));
  for (uint8_t ch = 0; ch < 4; ++ch) {
    if (!muxSelect(ch)) { Serial.printf("    ch%u : mux select failed\n", ch); continue; }
    if (!ping(AS5600_ADDR)) { Serial.printf("    ch%u : no AS5600\n", ch); continue; }
    uint8_t st = 0, agc = 0;
    readReg8(AS5600_ADDR, AS5600_STATUS, st);
    readReg8(AS5600_ADDR, AS5600_AGC, agc);
    const bool md = st & 0x20, ml = st & 0x10, mh = st & 0x08;
    Serial.printf("    ch%u : AS5600 angle=%4d agc=%3u  %s\n", ch, readAngle(ch), agc,
                  !md ? "NO MAGNET" : ml ? "magnet too WEAK (move closer)"
                                  : mh ? "magnet too STRONG (move away)" : "magnet ok");
  }
  muxDisable();
}

// The heart of it: spin one motor, watch every encoder and every current
// channel. Whichever encoder moves is that motor's wheel.
static void mapMotors() {
  Serial.println(F("\n--- MOTOR MAPPING --- WHEELS OFF THE GROUND. Any key aborts.\n"));

  // Probe the mux ONCE. Without it every readAngle() NACKs, and the I2C driver
  // prints four lines of error per failure -- roughly 400 lines that bury the
  // current columns, which are the part that still works. A missing subsystem
  // should be reported once, not screamed on every access.
  const bool have_enc = ping(pins::kMuxAddr);
  if (!have_enc)
    Serial.println(F("NOTE: no mux at 0x70 -- encoder columns will read 0.\n"
                     "      Current pairing below is still valid.\n"));

  Serial.println(F("motor | encoder deltas (ch0..ch3)      | current mV (A0..A3)"));
  Serial.println(F("------+-------------------------------+---------------------"));

  for (int m = 0; m < 4; ++m) {
    int before[4] = {-1, -1, -1, -1};
    if (have_enc) for (uint8_t ch = 0; ch < 4; ++ch) before[ch] = readAngle(ch);

    // Baseline is an AVERAGE, not one sample. At 25% duty off the ground a motor
    // pulls only a few hundred mA, which at 26.4 mV/A is single-digit millivolts
    // against an ACS758 noise floor of ~4.6 mV. A single baseline sample carries
    // that whole noise into every later comparison.
    long idle[4] = {0, 0, 0, 0};
    for (uint8_t c = 0; c < 4; ++c) {
      long acc = 0;
      for (int k = 0; k < 8; ++k) acc += readAdsMv(c);
      idle[c] = acc / 8;
    }

    motorRun(m, SPIN_DUTY_PCT, false);
    const uint32_t t0 = millis();
    // MEAN over the spin window, not peak-hold. Peak latches the single highest
    // sample, so it reports the largest NOISE excursion whenever the signal sits
    // near the floor -- exactly this regime. The mean converges on the real draw.
    long acc[4] = {0, 0, 0, 0};
    long n[4]   = {0, 0, 0, 0};
    while (millis() - t0 < SPIN_MS) {
      for (uint8_t c = 0; c < 4; ++c) {
        const long mv = readAdsMv(c);
        if (mv >= 0) { acc[c] += mv; ++n[c]; }
      }
      if (Serial.available()) { stopAll(); Serial.println(F("\naborted")); return; }
    }
    long mean[4];
    for (uint8_t c = 0; c < 4; ++c) mean[c] = n[c] ? acc[c] / n[c] : 0;
    int after[4] = {-1, -1, -1, -1};
    if (have_enc) for (uint8_t ch = 0; ch < 4; ++ch) after[ch] = readAngle(ch);
    motorStop(m);

    Serial.printf(" SV%-2u  |", pins::kWheelSV[m]);
    int best = -1, best_abs = 0;
    for (uint8_t ch = 0; ch < 4; ++ch) {
      const int d = (before[ch] < 0 || after[ch] < 0) ? 0 : angleDelta(before[ch], after[ch]);
      if (abs(d) > best_abs) { best_abs = abs(d); best = ch; }
      Serial.printf(" %+6d", d);
    }
    Serial.print(F("  |"));
    for (uint8_t c = 0; c < 4; ++c) Serial.printf(" %5ld", mean[c] - idle[c]);
    if (!have_enc)                       Serial.print(F("   <- current only (no mux)"));
    else if (best >= 0 && best_abs > 20) Serial.printf("   <- encoder ch%d", best);
    else                                 Serial.print(F("   <- NO ENCODER MOVED"));
    Serial.println();
    delay(800);
  }
  stopAll();
  Serial.println(F("\nEach row is one corner: motor SV pin, its encoder channel, its current channel."));
}

// All four VL53L0X boot at 0x29, so they can only be separated by holding every
// sensor in shutdown and releasing them ONE AT A TIME, re-addressing each before
// the next is allowed to speak. XSHUT is active-low shutdown; driving it high
// releases the part. Wiring is the netlist's: pins::kTofXSHUT = {4,5,6,7}.
static bool tofBringUp() {
  for (int i = 0; i < 4; ++i) {
    pinMode(pins::kTofXSHUT[i], OUTPUT);
    digitalWrite(pins::kTofXSHUT[i], LOW);        // everyone in shutdown
    g_tof_ok[i] = false;
  }
  delay(20);

  Serial.println(F("\n--- ToF BRING-UP (XSHUT, one at a time) ---"));
  int n_ok = 0;
  for (int i = 0; i < 4; ++i) {
    digitalWrite(pins::kTofXSHUT[i], HIGH);       // release just this one
    delay(20);
    g_tof[i].setTimeout(500);
    if (!g_tof[i].init()) {
      Serial.printf("  XSHUT GPIO%-2u : init FAILED (nothing answered at 0x29)\n",
                    pins::kTofXSHUT[i]);
      digitalWrite(pins::kTofXSHUT[i], LOW);      // park it so it cannot collide
      continue;
    }
    const uint8_t addr = static_cast<uint8_t>(0x30 + i);
    g_tof[i].setAddress(addr);                    // move it off 0x29
    g_tof[i].startContinuous();
    g_tof_ok[i] = true;
    ++n_ok;
    Serial.printf("  XSHUT GPIO%-2u : ok -> re-addressed to 0x%02X\n",
                  pins::kTofXSHUT[i], addr);
  }
  Serial.printf("  %d/4 sensors up\n", n_ok);
  return n_ok > 0;
}

// Identification is physical: stream all four ranges and let the operator cover
// one sensor at a time. Whichever column collapses is that XSHUT pin's sensor,
// which is how an index in pins.h becomes a corner of the actual robot.
static void tofIdentify() {
  if (!tofBringUp()) {
    Serial.println(F("  no sensors -- check 3V3 at the modules and the XSHUT wiring"));
    return;
  }
  Serial.println(F("\n  COVER ONE SENSOR AT A TIME and watch which column drops."));
  Serial.println(F("  Any key stops.\n"));
  Serial.printf("   GPIO%-6u GPIO%-6u GPIO%-6u GPIO%u\n", pins::kTofXSHUT[0],
                pins::kTofXSHUT[1], pins::kTofXSHUT[2], pins::kTofXSHUT[3]);
  while (!Serial.available()) {
    for (int i = 0; i < 4; ++i) {
      if (!g_tof_ok[i]) { Serial.print(F("      --- ")); continue; }
      const uint16_t mm = g_tof[i].readRangeContinuousMillimeters();
      if (g_tof[i].timeoutOccurred()) Serial.print(F("    TIMEOUT"));
      else                            Serial.printf(" %6u mm", mm);
    }
    Serial.println();
    delay(200);
  }
  while (Serial.available()) Serial.read();
  Serial.println(F("  stopped"));
}

// Exhaustive hunt for the encoders: every address on every one of the eight mux
// channels, not just 0x36 on 0..3.
//
// Devices on the TRUNK stay visible with a channel open -- they sit upstream of
// the switch -- so a naive per-channel scan reports the ADS1115 and the IMU
// eight times and buries anything real. Baseline the trunk first with all
// channels closed, then report only what is NEW on each channel. What prints is
// then, by construction, actually behind that channel.
static void huntEncoders() {
  Serial.println(F("\n--- ENCODER HUNT: all 8 channels, all addresses ---"));
  if (!ping(pins::kMuxAddr)) {
    Serial.println(F("  no mux at 0x70 -- nothing to sweep"));
    return;
  }

  muxDisable();
  bool trunk[0x78] = {false};
  Serial.print(F("  trunk (channels closed):"));
  for (uint8_t a = 0x08; a <= 0x77; ++a) {
    if (ping(a)) { trunk[a] = true; Serial.printf(" 0x%02X", a); }
  }
  Serial.println();

  int total = 0;
  for (uint8_t ch = 0; ch < 8; ++ch) {
    if (!muxSelect(ch)) { Serial.printf("  ch%u : SELECT FAILED\n", ch); continue; }
    int found = 0;
    for (uint8_t a = 0x08; a <= 0x77; ++a) {
      if (trunk[a] || a == pins::kMuxAddr) continue;   // upstream, not behind this channel
      if (!ping(a)) continue;
      if (!found) Serial.printf("  ch%u :", ch);
      Serial.printf(" 0x%02X%s", a, a == AS5600_ADDR ? " (AS5600)" : "");
      ++found;
    }
    if (found) { Serial.println(); total += found; }
  }
  muxDisable();

  if (total == 0) {
    Serial.println(F("  NOTHING behind any channel. The mux answers, so the trunk and the"));
    Serial.println(F("  channel select both work -- the fault is downstream of the switch:"));
    Serial.println(F("    1. 3V3 actually present at each AS5600 module?"));
    Serial.println(F("    2. SDA/SCL swapped between mux SDn/SCn and the encoder?"));
    Serial.println(F("    3. pull-ups on the branch? the switch does not provide them"));
    Serial.println(F("    4. connector reversed (VCC<->GND)? that also runs hot"));
  } else {
    Serial.printf("  %d device(s) found behind channels\n", total);
  }
}

static void help() {
  Serial.println(F(
      "\n  s  I2C inventory (safe, no motion)\n"
      "  m  motor mapping -- SPINS EACH WHEEL 1.5 s at 10%\n"
      "  t  ToF identify\n"
      "  x  stop everything\n"));
}

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);          // never wedge on a closed terminal
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  // A NACK is how this tool ASKS "is anything there?" -- the driver logging four
  // error lines per absent device buried the working columns in ~400 lines.
  esp_log_level_set("i2c.master", ESP_LOG_NONE);

  motorInit();
  Wire.begin(pins::kI2C_SDA, pins::kI2C_SCL, 400000);

  Serial.println(F("\n\n========== BASE IDENTIFICATION =========="));
  Serial.printf("SDA=GPIO%u SCL=GPIO%u  mux 0x%02X\n",
                pins::kI2C_SDA, pins::kI2C_SCL, pins::kMuxAddr);
  inventory();
  help();
}

void loop() {
  if (!Serial.available()) { delay(20); return; }
  switch (Serial.read()) {
    case 's': inventory(); break;
    case 'm': mapMotors(); break;
    case 'e': huntEncoders(); break;
    case 't': tofIdentify(); break;
    case 'x': stopAll(); Serial.println(F("stopped")); break;
    case '?': help(); break;
    default: break;
  }
}
