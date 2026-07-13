#pragma once
#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

#include "bench_drive.h"

// Shared 2-sensor edge-align state machine (ORIENT -> CENTER_X) for the bench
// rig. PURE LOGIC: reads the two up-facing front ToF sensors and drives the
// motors via bench_drive.h; it does NOT print, so the identical code runs in the
// serial build (Arduino Serial frontend) and the Bluepad32 gamepad build
// (Console frontend). A frontend calls alignSetup() once, alignSetAuto() on its
// enable/disable buttons, and alignUpdate(vx,vy,w) every loop, then reads the
// accessors to display state however it likes.
//
// Header-only (C++17 inline); include from exactly one .cpp per build.

// ---- ToF pins / addressing --------------------------------------------------
inline const uint8_t PIN_SDA = 38, PIN_SCL = 39;
inline const uint8_t PIN_XSHUT_A = 40, PIN_XSHUT_B = 41;
inline const uint8_t MUX_ADDR = 0x70;

// ---- Alignment tunables (bench-tune to the real mounting height) ------------
inline const uint16_t BAND_MIN_MM = 20; // board "present" when reading in [min,max]
inline const uint16_t BAND_MAX_MM = 500;
inline const uint8_t DEBOUNCE_N = 2;
inline const int ALIGN_ROT_DUTY = 180;           // ORIENT rotate duty
inline const int CENTER_DUTY = 180;              // CENTER_X creep duty
inline const float CREEP_NOMINAL_MMPS = 150.0f;  // odometry scale (cancels at midpoint)
inline const float CENTER_TOL_MM = 12.0f;
inline const float FRONT_OFFSET_MM = 0.0f;
inline const uint32_t ORIENT_TIMEOUT_MS = 10000;
inline const uint32_t LOST_TIMEOUT_MS = 1500;
inline const uint32_t CENTER_TIMEOUT_MS = 12000;

// ---- ToF --------------------------------------------------------------------
inline VL53L0X tofA, tofB;
inline bool okA = false, okB = false;
inline bool useMux = false;

inline void muxSelect(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write((uint8_t)(1u << ch));
  Wire.endTransmission();
}
inline bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}
inline bool initOne(VL53L0X &s) {
  s.setTimeout(500);
  bool ok = false;
  for (int i = 0; i < 3 && !ok; ++i)
    ok = s.init();
  if (ok) {
    s.setMeasurementTimingBudget(33000);
    s.startContinuous();
  }
  return ok;
}
// Bring up both front sensors: mux (ch0/ch1) if present, else XSHUT (A@0x30, B@0x29).
inline void alignSetup() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  useMux = i2cPresent(MUX_ADDR);
  if (useMux) {
    muxSelect(0);
    okA = initOne(tofA);
    muxSelect(1);
    okB = initOne(tofB);
    return;
  }
  pinMode(PIN_XSHUT_A, OUTPUT);
  pinMode(PIN_XSHUT_B, OUTPUT);
  digitalWrite(PIN_XSHUT_A, LOW);
  digitalWrite(PIN_XSHUT_B, LOW);
  delay(20);
  digitalWrite(PIN_XSHUT_A, HIGH);
  delay(20);
  okA = initOne(tofA);
  if (okA)
    tofA.setAddress(0x30);
  digitalWrite(PIN_XSHUT_B, HIGH);
  delay(20);
  okB = initOne(tofB);
}
// Range in mm; sets *valid. Rejects no-target/signal-fail glitches at the source:
// only device range status 11 is a valid measurement (0x14 = RESULT_RANGE_STATUS).
inline uint16_t readOne(VL53L0X &s, bool ok, uint8_t muxCh, bool *valid) {
  if (!ok) {
    *valid = false;
    return 0xFFFF;
  }
  if (useMux)
    muxSelect(muxCh);
  const uint16_t mm = s.readRangeContinuousMillimeters();
  const uint8_t rangeStatus = (s.readReg(0x14) & 0x78) >> 3;
  *valid = !s.timeoutOccurred() && rangeStatus == 11 && mm < 8000;
  return mm;
}

// ---- Presence (band + debounce) ---------------------------------------------
struct Presence {
  bool present = false;
  uint8_t disagree = 0;
  void update(uint16_t mm, bool valid) {
    const bool raw = valid && mm >= BAND_MIN_MM && mm <= BAND_MAX_MM;
    if (raw == present)
      disagree = 0;
    else if (++disagree >= DEBOUNCE_N) {
      present = raw;
      disagree = 0;
    }
  }
};
inline Presence presA, presB;

// ---- Mode / state machine ---------------------------------------------------
enum class Mode { Manual, Orient, CenterSeek, CenterReturn, Centered };
inline Mode mode = Mode::Manual;
inline uint32_t mode_since = 0;
inline uint32_t both_lost_since = 0;
inline bool prev_any = false;
inline bool armed = false;
inline bool auto_trigger = false; // OFF by default; A enables, B disables

// cached readings for the frontend's status display
inline uint16_t g_mmA = 0, g_mmB = 0;
inline bool g_vA = false, g_vB = false;

// dead-reckon odometry for CENTER_X (only the ratio matters -> scale cancels)
inline float odom_x = 0.0f, x_far = 0.0f;
inline uint32_t odo_last_ms = 0;
inline void odoReset() {
  odom_x = 0.0f;
  odo_last_ms = millis();
}
inline void odoStep(int vx_sign) {
  const uint32_t now = millis();
  const float dt = (now - odo_last_ms) / 1000.0f;
  odo_last_ms = now;
  odom_x += vx_sign * CREEP_NOMINAL_MMPS * dt;
}

inline void setMode(Mode m) {
  mode = m;
  mode_since = millis();
  both_lost_since = 0;
}
inline void startOrient() { setMode(Mode::Orient); }
inline void abortToManual() {
  stopAll();
  setMode(Mode::Manual);
}

// A enables auto-align; B disables (and aborts any align in progress).
inline void alignSetAuto(bool on) {
  auto_trigger = on;
  if (!on && mode != Mode::Manual)
    abortToManual();
}
inline void alignStartManual() { // serial 'g' one-shot
  if (mode == Mode::Manual)
    startOrient();
}

// ---- One iteration: read sensors, run the state machine, drive the motors ---
inline void alignUpdate(float vx, float vy, float w) {
  bool vA = false, vB = false;
  const uint16_t mmA = readOne(tofA, okA, 0, &vA);
  const uint16_t mmB = readOne(tofB, okB, 1, &vB);
  presA.update(mmA, vA);
  presB.update(mmB, vB);
  g_mmA = mmA;
  g_vA = vA;
  g_mmB = mmB;
  g_vB = vB;
  const bool a = presA.present, b = presB.present, any = a || b;

  // Any manual stick input yields control back to manual.
  const bool manualActive = (vx != 0.0f || vy != 0.0f || w != 0.0f);
  if (manualActive && mode != Mode::Manual)
    setMode(Mode::Manual);

  if (!any)
    armed = true; // saw genuine clear space -> arm (kills boot/no-target glitch)
  if (mode == Mode::Manual && auto_trigger && armed && any && !prev_any)
    startOrient();
  prev_any = any;

  switch (mode) {
  case Mode::Manual:
    driveMixF(vx, vy, w, g_speed);
    break;
  case Mode::Orient:
    if (a && b) {
      stopAll();
      odoReset();
      setMode(Mode::CenterSeek);
    } else {
      const int rot = (a && !b) ? -1 : +1; // toward the lagging sensor
      driveMix(0, 0, rot, ALIGN_ROT_DUTY);
      if (!a && !b) {
        if (both_lost_since == 0)
          both_lost_since = millis();
        else if (millis() - both_lost_since > LOST_TIMEOUT_MS)
          abortToManual();
      } else {
        both_lost_since = 0;
      }
      if (millis() - mode_since > ORIENT_TIMEOUT_MS)
        abortToManual();
    }
    break;
  case Mode::CenterSeek:
    driveMix(1, 0, 0, CENTER_DUTY);
    odoStep(+1);
    if (!a && !b) {
      x_far = odom_x;
      setMode(Mode::CenterReturn);
    } else if (millis() - mode_since > CENTER_TIMEOUT_MS) {
      abortToManual();
    }
    break;
  case Mode::CenterReturn: {
    const float target = 0.5f * x_far - FRONT_OFFSET_MM;
    const float err = target - odom_x;
    if (fabsf(err) <= CENTER_TOL_MM) {
      stopAll();
      setMode(Mode::Centered);
    } else {
      const int dir = (err > 0) ? +1 : -1;
      driveMix(dir, 0, 0, CENTER_DUTY);
      odoStep(dir);
      if (millis() - mode_since > CENTER_TIMEOUT_MS)
        abortToManual();
    }
    break;
  }
  case Mode::Centered:
    stopAll();
    break;
  }
}

// ---- Accessors for a frontend's status display ------------------------------
inline const char *alignModeName() {
  switch (mode) {
  case Mode::Manual: return "MANUAL";
  case Mode::Orient: return "ORIENT";
  case Mode::CenterSeek: return "CENTER_X:seek";
  case Mode::CenterReturn: return "CENTER_X:return";
  case Mode::Centered: return "CENTERED";
  }
  return "?";
}
inline bool alignAutoEnabled() { return auto_trigger; }
inline uint16_t alignMM(uint8_t i) { return i == 0 ? g_mmA : g_mmB; }
inline bool alignValid(uint8_t i) { return i == 0 ? g_vA : g_vB; }
inline bool alignPresent(uint8_t i) { return i == 0 ? presA.present : presB.present; }
inline float alignOdomX() { return odom_x; }
