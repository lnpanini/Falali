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

// ---- Alignment tunables (bench-tune to the real mounting height) ------------
inline const uint16_t BAND_MIN_MM = 20; // board "present" when reading in [min,max]
inline const uint16_t BAND_MAX_MM = 500;
inline const uint8_t DEBOUNCE_N = 2; // 2 agreeing samples to flip (noise rejection restored)
inline const int ALIGN_ROT_DUTY = 90;            // ORIENT rotate duty (gentle -> less overshoot)
inline const int CENTER_DUTY = 90;               // CENTER_X creep duty (90 sustains straight motion; 70 stalled under load)
inline const int APPROACH_DUTY = 90;             // A-triggered forward approach (90 sustains; 70 stalled under load)
inline const float CREEP_NOMINAL_MMPS = 150.0f;  // odometry scale (cancels at midpoint)
inline const float CENTER_TOL_MM = 12.0f;
inline const float FRONT_OFFSET_MM = 0.0f;
inline const uint32_t ORIENT_TIMEOUT_MS = 10000;
inline const uint32_t LOST_TIMEOUT_MS = 1500;
inline const uint32_t CENTER_TIMEOUT_MS = 12000;
inline const uint32_t APPROACH_TIMEOUT_MS = 8000; // give up creeping if no edge is found
inline const float MISALIGN_TOL_MM = 15.0f;       // far-edge skew above this -> backtrack + re-align
inline const int MAX_REALIGN_RETRIES = 2;         // then accept & center (no infinite retry)

// Anti-stall: before each align move-from-rest, settle to a symmetric friction
// state (PAUSE), then a brief full-duty pulse to break stiction (KICK), so both
// sides launch together instead of one lurching first.
inline const uint32_t PAUSE_MS = 150;
inline const uint32_t KICK_MS = 80;
inline const int KICK_DUTY = 255;
// Edge-hold recovery: if the ORIENT jerk knocks the leading sensor out past the
// edge (BOTH sensors go absent), inch straight forward at this duty to slide them
// back under the board. Pure translation -> all wheels equal duty -> no stall.
inline const int RECOVER_DUTY = 110;
// Pivot compensation: bias ORIENT to rotate about the *leading* (already-detected)
// sensor instead of the robot centre, so it doesn't drift off the edge during the
// correction. Forward-bias gain = c / L_char = 26.25 / (68.75 + 82.5) ~= 0.17.
// TEMPORARILY DISABLED (=0) to isolate a bench fault: with it on, the b&&!a case
// both rotated off the edge (vx_comp sign didn't mirror between the two cases) and
// starved the back-left wheel to 64 duty (stall). Prove pure symmetric rotation is
// correct first, THEN re-enable with a hardware-verified, properly mirrored sign.
inline const float PIVOT_GAIN = 0.0f;

// ---- ToF --------------------------------------------------------------------
inline VL53L0X tofA, tofB;
inline bool okA = false, okB = false;
inline bool initOne(VL53L0X &s) {
  s.setTimeout(500);
  bool ok = false;
  for (int i = 0; i < 3 && !ok; ++i)
    ok = s.init();
  if (ok) {
    s.setMeasurementTimingBudget(33000); // 33ms: stable reads (20ms was too noisy for N=2)
    s.startContinuous();
  }
  return ok;
}
// Bring up both front sensors by XSHUT re-addressing (A@0x30, B@0x29). A
// TCA9548A mux branch was tried here and dropped 2026-08-31 -- the final design
// uses XSHUT on every board.
inline void alignSetup() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000); // 100kHz: reliable on the bench wiring (400k dragged the loop)
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
// Non-blocking range read: consumes a sample ONLY when the sensor signals one is
// ready (RESULT_INTERRUPT_STATUS), otherwise returns the cached value immediately
// so the control loop is never gated by the ~20ms integration (the stock
// readRangeContinuousMillimeters() spins until a fresh sample -- that spin was
// most of our sensing-to-logic latency). Range status 11 = a valid measurement
// (rejects no-target/signal-fail glitches at the source). *fresh tells the caller
// whether cachedMm/cachedValid were just updated (so it can debounce on real
// samples, not on cached repeats).
inline uint16_t readOne(VL53L0X &s, bool ok, uint16_t &cachedMm,
                        bool &cachedValid, bool *fresh) {
  *fresh = false;
  if (!ok) {
    cachedValid = false;
    return 0xFFFF;
  }
  if ((s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) == 0)
    return cachedMm; // no new sample yet -> don't block, reuse last reading
  const uint16_t mm = s.readReg16Bit(VL53L0X::RESULT_RANGE_STATUS + 10);
  const uint8_t rangeStatus = (s.readReg(VL53L0X::RESULT_RANGE_STATUS) & 0x78) >> 3;
  s.writeReg(VL53L0X::SYSTEM_INTERRUPT_CLEAR, 0x01); // let the next sample latch
  cachedValid = (rangeStatus == 11 && mm < 8000);
  cachedMm = mm;
  *fresh = true;
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
enum class Mode { Manual, Approach, Orient, CenterSeek, CenterReturn, Backtrack, Centered };
inline Mode mode = Mode::Manual;
inline uint32_t mode_since = 0;
inline uint32_t both_lost_since = 0;
inline bool auto_trigger = false; // armed by A, disarmed by B; OFF at boot

// Far-edge skew detection + auto-realign bookkeeping.
inline float seek_x_first = 0.0f;       // odom_x when the FIRST sensor cleared the far edge
inline bool seek_first_recorded = false;
inline int realign_retries = 0;         // re-align attempts this dock (capped)
inline bool reentered = false;          // Backtrack: board re-entered on the way out

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
// Integrate commanded motion. Scaled by duty/CENTER_DUTY so a full-duty kickstart
// burst is counted at roughly its real (faster) speed, keeping the CENTER_X
// midpoint consistent between the seek and the return. Call every loop (vx_sign=0
// while paused) so the time base stays current.
inline void odoStep(int vx_sign, int duty) {
  const uint32_t now = millis();
  const float dt = (now - odo_last_ms) / 1000.0f;
  odo_last_ms = now;
  odom_x += vx_sign * CREEP_NOMINAL_MMPS * (duty / (float)CENTER_DUTY) * dt;
}

// Move-from-rest sequencer: 0 while settling (caller stops), full duty during the
// kickstart window, then the cruise duty. `elapsed` is time since the phase began.
inline int phaseDuty(uint32_t elapsed, int cruise) {
  if (elapsed < PAUSE_MS)
    return 0; // settle to a symmetric friction state
  if (elapsed < PAUSE_MS + KICK_MS)
    return KICK_DUTY; // symmetric kickstart to break stiction
  return cruise;      // cruise
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
  // Non-blocking reads: consume a sample only when one is ready, else reuse the
  // cached g_mm*/g_v* (which double as the frontend's display values) so the loop
  // runs at full speed instead of stalling on the integration. Debounce only on
  // fresh samples so DEBOUNCE_N counts real sensor updates, not loop iterations.
  bool freshA = false, freshB = false;
  readOne(tofA, okA, g_mmA, g_vA, &freshA);
  readOne(tofB, okB, g_mmB, g_vB, &freshB);
  if (freshA)
    presA.update(g_mmA, g_vA);
  if (freshB)
    presB.update(g_mmB, g_vB);
  const bool a = presA.present, b = presB.present, any = a || b;

  // A arms (auto_trigger). Arming starts the autonomous dock: if already on an edge
  // go straight to ORIENT, otherwise slow-APPROACH forward until the first edge.
  // From there stick input is ignored (the auto-mode switch cases never read
  // vx/vy/w) and B is the only way out (alignSetAuto(false) -> abortToManual()).
  // Boot/no-target glitches are rejected at the sensor (range-status 11 in readOne).
  if (mode == Mode::Manual && auto_trigger) {
    realign_retries = 0; // fresh dock -> reset the re-align budget
    if (any)
      startOrient();
    else
      setMode(Mode::Approach);
  }

  switch (mode) {
  case Mode::Manual:
    driveMixF(vx, vy, w, g_speed);
    break;
  case Mode::Approach: {
    // A-triggered slow creep toward the board; hand to ORIENT at the first edge.
    if (any) {
      stopAll();
      startOrient();
      break;
    }
    const int duty = phaseDuty(millis() - mode_since, APPROACH_DUTY);
    if (duty == 0) { // settle/kick from rest
      stopAll();
      break;
    }
    driveMix(1, 0, 0, duty);
    if (millis() - mode_since > APPROACH_TIMEOUT_MS)
      abortToManual(); // crept the full timeout with no edge -> give up
    break;
  }
  case Mode::Orient: {
    if (a && b) {
      stopAll();
      odoReset();
      seek_first_recorded = false; // start a fresh far-edge skew measurement
      setMode(Mode::CenterSeek);
      break;
    }
    const int duty = phaseDuty(millis() - mode_since, ALIGN_ROT_DUTY);
    if (duty == 0) { // settle first (symmetric stiction break)
      stopAll();
      break;
    }
    // Edge-hold recovery: the rotation jerk can fling the leading sensor out past
    // the edge, dropping BOTH to absent. Don't spin blindly -- inch straight
    // forward (pure translation -> all wheels equal duty -> no stall) to slide the
    // sensors back under the board; rotation resumes the instant one re-catches.
    if (!a && !b) {
      if (both_lost_since == 0)
        both_lost_since = millis();
      driveMix(1, 0, 0, RECOVER_DUTY);
      if (millis() - both_lost_since > LOST_TIMEOUT_MS)
        abortToManual(); // inched forward and still never re-caught -> give up
      break;
    }
    both_lost_since = 0;
    // Exactly one sensor present: rotate toward the lagging sensor (pure spin;
    // pivot-comp retired -- the recovery-inch above solves edge-drift instead).
    const float rot = (a && !b) ? -1.0f : +1.0f;
    driveMixF(0.0f, 0.0f, rot, duty);
    if (millis() - mode_since > ORIENT_TIMEOUT_MS)
      abortToManual();
    break;
  }
  case Mode::CenterSeek: {
    const int duty = phaseDuty(millis() - mode_since, CENTER_DUTY);
    if (duty == 0) { // settle
      stopAll();
      odoStep(0, 0);
      break;
    }
    driveMix(1, 0, 0, duty);
    odoStep(+1, duty);
    // Far-edge skew = along-track gap between the two sensors clearing the far edge
    // (~52.5mm * tan(yaw)); a big gap means the straight drive drifted off-square.
    // Record the first loss; if it re-acquires before the second, it was noise.
    if ((a != b) && !seek_first_recorded) {
      seek_first_recorded = true;
      seek_x_first = odom_x;
    } else if (a && b) {
      seek_first_recorded = false;
    }
    if (!a && !b) {
      x_far = odom_x;
      const float skew = seek_first_recorded ? (x_far - seek_x_first) : 0.0f;
      if (skew > MISALIGN_TOL_MM && realign_retries < MAX_REALIGN_RETRIES) {
        realign_retries++;
        reentered = false;
        setMode(Mode::Backtrack); // crossed far edge off-square -> back out & retry
      } else {
        setMode(Mode::CenterReturn); // square enough (or out of retries) -> center
      }
    } else if (millis() - mode_since > CENTER_TIMEOUT_MS) {
      abortToManual();
    }
    break;
  }
  case Mode::CenterReturn: {
    const int duty = phaseDuty(millis() - mode_since, CENTER_DUTY);
    if (duty == 0) { // settle before reversing -> clean stiction break
      stopAll();
      odoStep(0, 0);
      break;
    }
    const float target = 0.5f * x_far - FRONT_OFFSET_MM;
    const float err = target - odom_x;
    if (fabsf(err) <= CENTER_TOL_MM) {
      stopAll();
      setMode(Mode::Centered);
    } else {
      const int dir = (err > 0) ? +1 : -1;
      driveMix(dir, 0, 0, duty);
      odoStep(dir, duty);
      if (millis() - mode_since > CENTER_TIMEOUT_MS)
        abortToManual();
    }
    break;
  }
  case Mode::Backtrack: {
    // Far edge was crossed off-square: reverse straight out past the NEAR edge
    // (re-enter the board, then leave it), then re-run the slow approach so ORIENT
    // re-squares at the near edge. Pure -x translation -> all wheels equal -> no stall.
    const int duty = phaseDuty(millis() - mode_since, CENTER_DUTY);
    if (duty == 0) {
      stopAll();
      break;
    }
    driveMix(-1, 0, 0, duty);
    if (a && b)
      reentered = true; // back under the board
    if (reentered && !any) {
      stopAll();
      setMode(Mode::Approach); // backed out the near side -> fresh approach
    } else if (millis() - mode_since > CENTER_TIMEOUT_MS) {
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
  case Mode::Approach: return "APPROACH";
  case Mode::Orient: return "ORIENT";
  case Mode::CenterSeek: return "CENTER_X:seek";
  case Mode::CenterReturn: return "CENTER_X:return";
  case Mode::Backtrack: return "BACKTRACK";
  case Mode::Centered: return "CENTERED";
  }
  return "?";
}
inline bool alignAutoEnabled() { return auto_trigger; }
inline uint16_t alignMM(uint8_t i) { return i == 0 ? g_mmA : g_mmB; }
inline bool alignValid(uint8_t i) { return i == 0 ? g_vA : g_vB; }
inline bool alignPresent(uint8_t i) { return i == 0 ? presA.present : presB.present; }
inline float alignOdomX() { return odom_x; }
