/*
 * BLD-120A + BLDC motor bench bring-up  —  ESP32-WROOM-32D
 * =========================================================================
 * Single motor, serial-console control. Bench rig to validate the BLD-120A
 * control interface (SV / F-R / EN / BRK) before trusting the production S3
 * wiring in ../include/pins.h.
 *
 *   pio run -e bench_motor -t upload && pio device monitor -e bench_motor
 *
 * BLD-120A terminals (top→bottom on the green block):
 *     BRK  EN  F/R  COM  SV  REF+  HU HV HW  REF-  W  V  U  DC+  DC-
 *
 * ── Wiring (colour code as built on the bench, 2026-07-23) ───────────────
 *   The driver's first five terminals are BRK EN F/R COM SV, left to right,
 *   and the loom lands on them in that order:
 *
 *     brown   ESP GPIO18 ── BRK    brake       (LOW/COM = brake on)
 *     red     ESP GPIO17 ── EN     enable      (LOW/COM = enabled)
 *     orange  ESP GPIO16 ── F/R    direction   (HIGH/open = forward)
 *     black   ESP GND    ── COM    signal ground
 *     white   ESP GPIO25 ── SV     speed command, 0–3.3 V analog (DAC1)
 *
 *   COM is MANDATORY: it is the 0 V that SV and the EN/F-R/BRK levels
 *   reference (the speed pot returns to COM). Do NOT reference the ESP to
 *   DC- (motor return).
 *   REF+/REF- = +5 V / gnd for the MOTOR's hall sensors — leave the ESP off them.
 *   Motor: hall cable → HU HV HW + REF+/REF- ;  phase wires → U V W.
 *   Supply: DC+/DC- = +24 V bench PSU, on its own. ESP on USB. Never bridge them.
 *
 * ── AS5600 encoder on the motor output shaft (speed feedback) ────────────
 *   Upstream of the 15:1 gearbox, NOT on the wheel axle: measured RPM here is
 *   shaft RPM, 15x wheel RPM. Anyone converting counts to distance must divide
 *   by the gearbox ratio first.
 *   7-pin breakout.  Five wires land; two are deliberately left floating:
 *
 *     VCC ── ESP 3V3     NOT 5V: the board's I2C pull-ups sit on VCC, and a
 *                        5 V rail would hold SDA/SCL at 5 V into a chip whose
 *                        GPIOs are not 5 V tolerant.
 *     GND ── ESP GND
 *     SDA ── ESP GPIO21
 *     SCL ── ESP GPIO22
 *     DIR ── GND         hard-tied = CW-increasing. Never leave floating: DIR is
 *                        sampled continuously, so a floating pin makes the sign
 *                        flip on noise (looks like a wheel spinning backwards).
 *     OUT ── (floating)  analog/PWM angle — redundant while we read over I2C.
 *     GPO ── (floating)  OTP *programming* pin. Do not drive it; a stray burn of
 *                        ZPOS/MPOS/CONF is permanent and unrecoverable.
 *
 *   I2C is GPIO21/22 here, NOT the 38/39 in include/pins.h — those are S3 pins.
 *   On this chip GPIO39 is input-only (no output driver, so it can never pull
 *   SCL low) and GPIO38 is not bonded out on WROOM-32.
 *
 *   Address is 0x36, FIXED in silicon — no address pins. Four encoders cannot
 *   share one bus; scaling to 4 wheels needs the TCA9548A mux, one bus each, or
 *   the analog OUT pins.
 *
 * ── The 3.3 V ceiling (known limitation) ─────────────────────────────────
 *   SV wants 0–5 V. The ESP can only source 0–3.3 V, so full-scale command
 *   ≈ 66 % of the driver's span — and less under load, because SV is a
 *   pot-wiper input that drags the source down (bench-measured 2.59 V at full
 *   command on 2026-07-15, i.e. ~52 %). This rig therefore cannot reach top
 *   speed. It is fine for validating polarity/ramp/brake behaviour.
 *   To recover the full span later: an MCP4725 I2C DAC run off 5 V
 *   (Adafruit_MCP4725), or a non-inverting op-amp ×1.5 buffer on GPIO25.
 *
 * ── Onboard trims / indicator ────────────────────────────────────────────
 *   "Peak Power" knob (0.8–8.0 A) = driver current limit → position unverified
 *   within its range; there is no PSU current limit backing it up (see Safety
 *   net), so this trim is doing real work — do not assume it is set LOW.
 *   "RV"/"P-sv" = onboard speed trim; if SV seems ignored, turn it to max.
 *   RUN/ALM = LED only (no ALM terminal, no FG) → SV control is open-loop
 *   (not servoed from RPM); only fault sensing (the stall trip below) is
 *   closed-loop now.
 *
 * ── Safety net ───────────────────────────────────────────────────────────
 *   The software stall trip (StallDetector, armed in loop()) is now the
 *   primary automatic protection: commanded at/above break-away with |RPM| < 5
 *   for 250 ms trips it, collapsing SV and asserting brake. It reacts in
 *   ~250 ms and, unlike a fixed current limit, doesn't false-trip on inrush.
 *   Hardware backstops, worst case first: the driver's "Peak Power" trim
 *   (≤8 A, position unverified) → a 10 A fuse. The bench PSU has NO current
 *   limit of its own — an earlier version of this note claimed otherwise;
 *   that was a documentation error, corrected 2026-07-27. The fuse cannot
 *   protect the motor: an 8 A stall sits below the fuse rating indefinitely
 *   while ~192 W cooks stationary windings, so the software trip is doing
 *   the job the fuse can't. `x` is the manual stop.
 *   Watch the RUN/ALM LED for faults the software can't see (driver-side
 *   shorts, MOSFETs failed closed).
 *
 * ── Control-line electrical model ────────────────────────────────────────
 *   A COM terminal means EN/F-R/BRK activate by being pulled to COM, so they
 *   default to OPEN-DRAIN: LOW = pull to COM = assert, HIGH = release (Hi-Z),
 *   the driver's internal pull-up then holds the line high.
 *   If your board's inputs are plain 3.3 V logic instead, set PUSH_PULL_CTRL 1.
 *   If a control acts inverted at low duty, flip its *_ASSERT / _FWD constant.
 *
 * ── Console (Serial @115200) ─────────────────────────────────────────────
 *   Single keys fire INSTANTLY, no Enter needed (that includes the e-stop):
 *     e / d     enable / disable
 *     f / r     forward / reverse
 *     b / n     brake on / brake off (release)
 *     + / -     nudge command by 8
 *     x         E-STOP  (disable + brake + command 0)
 *     ?         help + status
 *   These need Enter (they take an argument):
 *     s <0-255> set speed command      e.g.  s 120
 *     v <volts> set speed by SV volts  e.g.  v 1.8
 *     sweep     auto-ramp 0 → max → 0, printing the response
 * =========================================================================
 */

#include <Arduino.h>
#include <SerialCommands.h>
#include <Wire.h>
#include <AS5600.h>
#include "StallDetector.h"
#include "MotorCalAnalysis.h"

// ── Pins (ESP32-WROOM-32D) — validated against the WROOM pin rules ────────
// GPIO16/17 are PSRAM on WROVER but FREE on WROOM-32D. GPIO2 is a strapping
// pin (must be low/floating at boot); it is the stock onboard LED, so driving
// it as a heartbeat is safe — just never hang a pull-up off it.
constexpr int PIN_SV  = 25;   // DAC1 — true analog out (also LEDC-capable)
constexpr int PIN_FR  = 16;
constexpr int PIN_EN  = 17;
constexpr int PIN_BRK = 18;
constexpr int PIN_LED = 2;    // onboard LED heartbeat
constexpr int PIN_SDA = 21;   // AS5600 — I2C0 default on this chip
constexpr int PIN_SCL = 22;

// ── AS5600 motor-shaft encoder ───────────────────────────────────────────
// 12-bit absolute-within-one-turn: 4096 counts per revolution, no index pulse
// and no hardware turn counter. Multi-turn distance therefore only exists if we
// keep calling getCumulativePosition() often enough to see each wrap — see the
// Nyquist note at the control tick.
constexpr int32_t ENC_CPR = 4096;

// AGC full scale is SUPPLY DEPENDENT: 0-255 at 5 V, but only 0-128 at 3.3 V.
// We power the breakout from 3V3 (see wiring above), so 128 is the rail, not
// the middle. Reading it against /255 makes a maxed-out gain look ideal.
constexpr uint8_t ENC_AGC_FS = 128;

// ── SV output mode ───────────────────────────────────────────────────────
// 1 = DAC1 true analog DC (default: no ripple, no reliance on the driver's
//     input filter to average a square wave).
// 0 = LEDC PWM, matching how the production firmware drives SV.
#define SV_USE_DAC 1

// ── Optional SV readback ─────────────────────────────────────────────────
// Verifies what the driver ACTUALLY sees vs what we commanded — this is how
// you catch the input-loading droop described above.
// Wire SV through a 10 k / 10 k divider to GPIO34 (input-only, ADC1) so the
// pin stays safe even if SV is ever driven to 5 V by an external buffer.
#define HAS_SV_SENSE 0
constexpr int   PIN_SV_SENSE   = 34;
constexpr float SV_SENSE_RATIO = 2.0f;   // (R1+R2)/R2 for a 10k/10k divider

// ── Control-line drive style & polarity ──────────────────────────────────
#define PUSH_PULL_CTRL 0   // 0 = open-drain (pull to COM); 1 = push-pull 3.3 V

// "Asserted" level per line. With open-drain, asserting = LOW (pulled to COM).
constexpr int EN_ASSERT  = LOW;   // EN low (to COM) = enabled      — CONFIRMED 2026-07-15
constexpr int FR_FWD     = HIGH;  // F/R HIGH (pulled up) = forward — CONFIRMED 2026-07-15
constexpr int BRK_ASSERT = LOW;   // BRK low (to COM) = brake       — CONFIRMED 2026-07-15

// ── Command scale ────────────────────────────────────────────────────────
// 0..255 in both modes: the DAC is natively 8-bit and the production LEDC
// duty is 8-bit, so `s 120` means exactly the same thing either way.
constexpr int      CMD_MAX   = 255;
constexpr float    ESP_VMAX  = 3.3f;   // what 255 actually produces
constexpr int      PWM_FREQ  = 1000;   // LEDC mode only — matches production
constexpr int      PWM_RES   = 8;
constexpr int      PWM_CH    = 0;
constexpr int      SLEW_STEP = 4;      // command units per tick → soft ramp
constexpr uint32_t TICK_MS   = 20;     // 50 Hz control tick
constexpr uint32_t ENC_TICK_MS = 4;    // 250 Hz encoder sampling — see loop()
constexpr uint32_t STATUS_MS = 500;    // 2 Hz status print

// ── State ────────────────────────────────────────────────────────────────
bool g_enabled  = false;
bool g_forward  = true;
bool g_brake    = false;
int  g_targetSv = 0;     // commanded 0..255
int  g_sv       = 0;     // actual (slewed) 0..255

// Auto-sweep
bool     g_sweep     = false;
int      g_sweepDir  = +1;
uint32_t g_sweepNext = 0;

// Encoder
AS5600   g_enc;
bool     g_encOk   = false;   // false => print "--" rather than a fake 0 RPM
int32_t  g_encPos  = 0;       // cumulative counts (signed, survives wraparound)
int32_t  g_rpmPos  = 0;       // position at the last RPM window boundary
uint32_t g_rpmTime = 0;
float    g_rpm     = 0.0f;

// Stall trip needs a faster RPM than the 500 ms status window: the trip fires at
// 250 ms, so a 500 ms estimate could not resolve it. 100 ms gives ~2.5 samples
// inside the trip window.
constexpr uint32_t RPM_FAST_MS = 100;
float    g_rpmFast   = 0.0f;
int32_t  g_fastPos   = 0;
uint32_t g_fastTime  = 0;

// Break-away defaults to 40 until calsweep measures the real value.
int g_breakAwayCmd = 40;
tb::StallDetector g_stall;

// ── Calibration: transfer curve ──────────────────────────────────────────
// Four legs, each 33 points at cmd = min(i*8, 255): FR=HIGH up, HIGH down,
// LOW up, LOW down. The down-legs are what yield stiction hysteresis, and the
// two FR states independently verify that the encoder sign follows F/R.
constexpr uint32_t CAL_SETTLE_MS  = 800;
constexpr uint32_t CAL_MEASURE_MS = 400;
constexpr int      CAL_POINTS     = 33;
constexpr float    GEAR_RATIO     = 15.0f;

enum class CalState : uint8_t { Idle, Settle, Measure, Done };

CalState g_cal      = CalState::Idle;
int      g_calLeg   = 0;      // 0..3
int      g_calIdx   = 0;      // 0..32 within the leg
uint32_t g_calUntil = 0;
int32_t  g_calPos0  = 0;
uint32_t g_calT0    = 0;
float    g_calPrevRpm = 0.0f;   // previous point in this leg, for sag detection

static int calCmdForIndex(int i) { const int c = i * 8; return c > 255 ? 255 : c; }
static bool calLegIsHigh(int leg) { return leg < 2; }
static bool calLegIsUp(int leg)   { return (leg % 2) == 0; }

// Ascending legs walk 0..32; descending legs walk 32..0.
static int calCmdForLeg(int leg, int idx) {
  return calCmdForIndex(calLegIsUp(leg) ? idx : (CAL_POINTS - 1 - idx));
}

// ── Calibration: step response ───────────────────────────────────────────
// Logs at the encoder sample rate, not the 2 Hz status rate — a time constant
// cannot be extracted from 2 samples per second.
constexpr uint32_t STEP_LOG_MS   = 4;
constexpr uint32_t STEP_RISE_MS  = 2000;
constexpr uint32_t STEP_STOP_MS  = 6000;   // cap on coast/brake captures
constexpr float    STEP_STOP_RPM = 5.0f;   // "stopped" threshold

enum class StepPhase : uint8_t { Idle, Rise, Settle, Coast, Brake, Done };

StepPhase g_step     = StepPhase::Idle;
int       g_stepCmd  = 0;
uint32_t  g_stepT0   = 0;
uint32_t  g_stepNext = 0;

// ── Encoder ──────────────────────────────────────────────────────────────
// Walk the bus and report every responder. This is the first thing to run when
// the encoder misbehaves: 0x36 present = wiring + pull-ups are good, and any
// remaining fault is the magnet. Nothing present = wiring.
static void i2cScan() {
  Serial.println(F("> I2C scan (GPIO21 SDA / GPIO22 SCL):"));
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("    0x%02X%s\n", addr,
                    addr == AS5600_DEFAULT_ADDRESS ? "  <- AS5600" : "");
      found++;
    }
  }
  if (!found) Serial.println(F("    (nothing responded — check SDA/SCL/3V3/GND)"));
}

// The STATUS register is the difference between "no magnet" and "magnet at the
// wrong height", which look identical from the angle reading alone.
static void encMagnetReport() {
  if (!g_encOk) { Serial.println(F("! encoder not connected")); return; }
  const uint8_t agc = g_enc.readAGC();
  Serial.printf("> magnet: %s%s%s  AGC:%u/%u  magnitude:%u\n",
                g_enc.magnetDetected()  ? "DETECTED" : "ABSENT",
                g_enc.magnetTooWeak()   ? " (TOO WEAK — move magnet CLOSER)" : "",
                g_enc.magnetTooStrong() ? " (TOO STRONG — move magnet AWAY)" : "",
                agc, ENC_AGC_FS, g_enc.readMagnitude());
  // AGC is the sensor's own gain, so it reads the airgap backwards: high gain
  // means it is straining to see a distant magnet. Aim for the middle.
  if (agc >= ENC_AGC_FS)
    Serial.println(F("  ! AGC railed at max gain — magnet too far (aim ~64)"));
  else if (agc <= 8)
    Serial.println(F("  ! AGC railed at min gain — magnet too close (aim ~64)"));
  else if (agc > (uint8_t)(ENC_AGC_FS * 3 / 4) || agc < (uint8_t)(ENC_AGC_FS / 4))
    Serial.println(F("  ~ AGC off-centre — usable, but tighten the airgap (aim ~64)"));
}

static void encBegin() {
  Wire.begin(PIN_SDA, PIN_SCL);
  // 400 kHz keeps a 250 Hz sample loop at ~4 % bus duty; the AS5600 is rated
  // well past this, and standard 100 kHz would make each read ~4x longer.
  Wire.setClock(400000);
  i2cScan();
  // begin() with no argument leaves DIR under hardware control, which is what we
  // want: DIR is hard-tied to GND on the breakout, not driven by a GPIO.
  g_enc.begin();
  g_encOk = g_enc.isConnected();
  Serial.printf("Encoder AS5600 @0x%02X: %s\n",
                AS5600_DEFAULT_ADDRESS, g_encOk ? "OK" : "NOT FOUND");
  if (g_encOk) {
    g_enc.resetCumulativePosition(0);
    encMagnetReport();
  }
}

// ── SV output ────────────────────────────────────────────────────────────
static void svBegin() {
#if SV_USE_DAC
  dacWrite(PIN_SV, 0);
#elif ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_SV, PWM_FREQ, PWM_RES);
#else
  ledcSetup(PWM_CH, PWM_FREQ, PWM_RES);
  ledcAttachPin(PIN_SV, PWM_CH);
#endif
}

static void svWrite(int cmd) {
#if SV_USE_DAC
  dacWrite(PIN_SV, (uint8_t)cmd);
#elif ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(PIN_SV, cmd);
#else
  ledcWrite(PWM_CH, cmd);
#endif
}

// ── Drive the control lines from current state ───────────────────────────
static void applyOutputs() {
  digitalWrite(PIN_EN,  g_enabled ? EN_ASSERT  : !EN_ASSERT);
  digitalWrite(PIN_FR,  g_forward ? FR_FWD     : !FR_FWD);
  digitalWrite(PIN_BRK, g_brake   ? BRK_ASSERT : !BRK_ASSERT);
}

static void printStatus() {
  const float svVolts = ESP_VMAX * g_sv / (float)CMD_MAX;
  Serial.printf("[t=%6.1fs] EN:%-3s DIR:%-3s BRK:%-3s cmd:%3d/255 -> %.2fV",
                millis() / 1000.0f,
                g_enabled ? "ON" : "off",
                g_forward ? "FWD" : "REV",
                g_brake ? "ON" : "off",
                g_sv, svVolts);
  // What fraction of the driver's real 0-5 V span this represents.
  Serial.printf(" (%2.0f%% of 0-5V)", 100.0f * svVolts / 5.0f);
#if HAS_SV_SENSE
  // analogReadMilliVolts applies the chip's factory ADC calibration — far more
  // accurate than scaling a raw analogRead() by hand.
  const float measured = analogReadMilliVolts(PIN_SV_SENSE) * SV_SENSE_RATIO / 1000.0f;
  Serial.printf("  MEAS:%.2fV", measured);
#endif
  // The only real feedback on this rig: the driver has no FG and no ALM.
  if (g_encOk) Serial.printf("  ENC:%+8.1fRPM %+7ldct", g_rpm, (long)g_encPos);
  else         Serial.print(F("  ENC:--"));
  Serial.println();
}

static void printHelp() {
  Serial.println(F(
    "\n-- BLD-120A motor bench (ESP32-WROOM-32D) -------------------\n"
    " instant keys (no Enter):\n"
    "   e/d enable/disable    f/r forward/reverse   b/n brake on/off\n"
    "   +/- nudge cmd +-8     x E-STOP              ? help+status\n"
    "   z   zero encoder      m magnet health\n"
    " with Enter:\n"
    "   s <0-255>  set command      v <volts>  set by SV volts\n"
    "   sweep      auto-ramp 0 -> max -> 0 (any key cancels)\n"
    "   scan       re-run the I2C bus scan\n"
    "-------------------------------------------------------------"));
}

static void setCommand(int v, const char* why) {
  const int before = g_targetSv;
  g_targetSv = constrain(v, 0, CMD_MAX);
  if (g_targetSv > before) g_stall.noteCommandIncrease(millis());
  Serial.printf("> %s: cmd %d/255 (~%.2fV)\n",
                why, g_targetSv, ESP_VMAX * g_targetSv / (float)CMD_MAX);
}

static void estop() {
  g_enabled = false;
  g_brake   = true;
  g_targetSv = 0;
  g_sv       = 0;          // no ramp-down on an e-stop: collapse SV immediately
  g_sweep    = false;
  g_cal      = CalState::Idle;
  g_step     = StepPhase::Idle;
  svWrite(0);
  applyOutputs();
  Serial.println(F("*** E-STOP: disabled + braked + command 0 ***"));
}

// Every calibration routine refuses to start for the same two reasons, and both
// otherwise present as "it just sits there": `x` latches the brake with no
// auto-clear, and without an encoder there is no measurement and no stall trip.
static bool calGuardOk(const char* what) {
  if (!g_encOk) {
    Serial.printf("! %s needs the encoder — none detected (run `scan`)\n", what);
    return false;
  }
  if (g_brake) {
    Serial.printf("! %s refused: brake is latched. Send `n` to release, then retry.\n", what);
    return false;
  }
  return true;
}

// A running calsweep or calstep owns g_targetSv/g_forward/g_brake for its
// full run and logs a CSV row per point/sample — the "commanded" columns are
// the routine's own bookkeeping, not a read of the live state. Any other
// command that touches those same variables (sweep's own g_targetSv writes,
// b/d/f/r, or the other calibration routine starting) races the run silently:
// nothing prints, but the CSV columns diverge from what the driver actually
// saw, and the whole run is corrupted with no indication anything went wrong.
// Both routines share this one guard so neither can clobber the other — a
// gate that only stopped calsweep-vs-others but let calstep start mid-sweep
// (or vice versa) would be worse than no gate at all, since it reads as
// protection while leaving that exact corruption path open.
// `x` (e-stop) is deliberately NOT gated by this — it must always be able to
// cut power.
static bool calBusy(const char* what) {
  const bool sweepBusy = (g_cal != CalState::Idle);
  const bool stepBusy  = (g_step != StepPhase::Idle && g_step != StepPhase::Done);
  if (!sweepBusy && !stepBusy) return false;
  Serial.printf("! %s running — send x to abort first (%s ignored)\n",
                sweepBusy ? "calsweep" : "calstep", what);
  return true;
}

// ── Console handlers ─────────────────────────────────────────────────────
// SerialCommands dispatches one-key commands on the FIRST character received,
// before any terminator — so `x` cuts the motor the instant the key is pressed
// rather than waiting for Enter. That is the whole reason for using it here.
static void cmdEnable (SerialCommands*) {
  g_enabled = true;
  g_stall.reset(millis());
  // reset() alone leaves ZERO grace (grace_until_ == now): g_sv has collapsed
  // to 0 while disabled and re-ramps from a standstill at SLEW_STEP/TICK_MS,
  // but g_targetSv may already be at/above break-away from before. Without a
  // fresh window, update() would see "commanded, not turning" on the very
  // next sample and trip on a motor that hasn't had any real chance to spin
  // up. noteCommandIncrease() must run AFTER reset(), since reset() is what
  // sets grace_until_ = now in the first place.
  g_stall.noteCommandIncrease(millis());
  Serial.println(F("> ENABLE"));
}
static void cmdDisable(SerialCommands*) {
  if (calBusy("d")) return;
  g_enabled = false; Serial.println(F("> DISABLE"));
}
static void cmdFwd    (SerialCommands*) {
  if (calBusy("f")) return;
  g_forward = true;  Serial.println(F("> FORWARD"));
}
static void cmdRev    (SerialCommands*) {
  if (calBusy("r")) return;
  g_forward = false; Serial.println(F("> REVERSE"));
}
static void cmdBrake  (SerialCommands*) {
  if (calBusy("b")) return;
  g_brake   = true;  Serial.println(F("> BRAKE on"));
}
static void cmdNoBrake(SerialCommands*) {
  g_brake = false;
  // Same rationale as cmdEnable(): releasing the brake lets g_sv re-ramp from
  // 0 toward whatever g_targetSv already is, so a fresh grace window is
  // needed or a stale/expired one lets the resume read as a locked rotor.
  // Unconditional is fine — below break-away update() already skips policing.
  g_stall.noteCommandIncrease(millis());
  Serial.println(F("> brake off"));
}
static void cmdUp     (SerialCommands*) { setCommand(g_targetSv + 8, "nudge up");   }
static void cmdDown   (SerialCommands*) { setCommand(g_targetSv - 8, "nudge down"); }
static void cmdEstop  (SerialCommands*) { estop(); }
static void cmdHelp   (SerialCommands*) { printHelp(); printStatus(); }
static void cmdMagnet (SerialCommands*) { encMagnetReport(); }
static void cmdScan   (SerialCommands*) { i2cScan(); }

static void cmdZero(SerialCommands*) {
  if (!g_encOk) { Serial.println(F("! encoder not connected")); return; }
  g_enc.resetCumulativePosition(0);
  g_encPos = g_rpmPos = 0;
  Serial.println(F("> encoder zeroed"));
}

static void cmdSet(SerialCommands* s) {
  const char* arg = s->Next();
  if (!arg) { Serial.println(F("? usage: s <0-255>")); return; }
  setCommand(atoi(arg), "set");
}

static void cmdVolts(SerialCommands* s) {
  const char* arg = s->Next();
  if (!arg) { Serial.println(F("? usage: v <volts, 0-3.3>")); return; }
  const float want = atof(arg);
  if (want > ESP_VMAX) {
    Serial.printf("! %.2fV exceeds the ESP's %.1fV ceiling — clamping\n", want, ESP_VMAX);
  }
  setCommand((int)lroundf(want / ESP_VMAX * CMD_MAX), "set by volts");
}

static void cmdSweep(SerialCommands*) {
  if (calBusy("sweep")) return;
  if (!g_enabled) { Serial.println(F("? enable first (e), then sweep")); return; }
  g_sweep = true; g_sweepDir = +1; g_sweepNext = 0;
  // Open the stall grace window ONCE here, at sweep start — do NOT move this into
  // the per-step ramp loop. sweep steps every 100 ms but StallConfig::grace_ms is
  // 300 ms: a call on every rising step (as a prior version did) re-extends
  // grace_until_ before the previous window has expired, so the detector never
  // leaves its grace window for nearly the whole ramp and the trip is effectively
  // disabled until the ramp ends — precisely when a locked rotor should have
  // tripped ~250 ms after crossing break-away. One window here is sufficient: the
  // motor measures 1012 RPM/V, so a healthy rotor clears the 5 RPM floor within
  // milliseconds of crossing break-away, and there is no legitimate point later in
  // the ramp where a working motor reads under 5 RPM. This single 300 ms window
  // covers only the initial break-away transient; the trip stays armed for the
  // rest of the ramp, which is the protection working as intended, not a risk.
  g_stall.noteCommandIncrease(millis());
  Serial.println(F("> SWEEP 0 -> max -> 0 (send x to abort)"));
}

static void cmdCalSweep(SerialCommands*) {
  // Re-invoking mid-run must refuse rather than silently restart from leg 0.
  if (calBusy("calsweep")) return;
  if (!calGuardOk("calsweep")) return;
  g_calLeg = 0; g_calIdx = 0; g_calPrevRpm = 0.0f;
  g_enabled = true;
  g_forward = true;                    // leg 0 is FR=HIGH
  g_stall.reset(millis());
  g_cal = CalState::Settle;
  g_calUntil = millis() + CAL_SETTLE_MS;
  setCommand(calCmdForLeg(0, 0), "calsweep");
  Serial.println(F("> CALSWEEP: 4 legs x 33 points, ~158s. Send x to abort."));
  Serial.println(F("CSV,sweep,fr_state,leg_dir,cmd,sv_volts,rpm_motor,rpm_wheel,counts_delta,agc,mag_status"));
}

static void cmdCalStep(SerialCommands* s) {
  // Same reasoning as cmdCalSweep(): refuse rather than silently restart, and
  // refuse if the OTHER calibration routine owns the state right now — see
  // calBusy()'s comment for why the guard has to cover both directions.
  if (calBusy("calstep")) return;
  if (!calGuardOk("calstep")) return;
  const char* arg = s->Next();
  if (!arg) { Serial.println(F("? usage: calstep <0-255>")); return; }
  g_stepCmd = constrain(atoi(arg), 0, CMD_MAX);
  if (g_stepCmd < g_breakAwayCmd) {
    Serial.printf("! cmd %d is below break-away (%d) — it will not turn\n",
                  g_stepCmd, g_breakAwayCmd);
    return;
  }
  g_enabled = true; g_forward = true; g_brake = false;
  g_stall.reset(millis());
  g_step   = StepPhase::Rise;
  g_stepT0 = millis();
  g_stepNext = g_stepT0;
  setCommand(g_stepCmd, "calstep rise");
  // Explicit, unconditional grace — do NOT rely on setCommand()'s own
  // noteCommandIncrease() call here. That call only fires when g_targetSv
  // actually INCREASES; if a prior manual `s`/`v` left g_targetSv at or above
  // g_stepCmd already, setCommand() sees no rise and opens no window, and
  // reset() just above already zeroed grace_until_ to now. Same failure mode
  // cmdEnable() guards against (see its comment) — call it directly so the
  // rise is always covered regardless of what g_targetSv held before.
  g_stall.noteCommandIncrease(millis());
  Serial.println(F("> CALSTEP: rise -> coast -> brake. Send x to abort."));
  Serial.println(F("CSV,step,phase,t_ms,cmd,rpm_motor"));
}

static void cmdUnknown(SerialCommands* s, const char* cmd) {
  s->GetSerial()->printf("? unknown '%s' — send ? for help\n", cmd);
}

// Terminator "\n" with "\r\n" also in the delimiter set, so the console works
// whether the monitor sends LF, CR or CRLF.
char g_cliBuf[32];
SerialCommands g_cli(&Serial, g_cliBuf, sizeof(g_cliBuf), "\n", " \r\n");

static SerialCommand c_enable ("e", cmdEnable,  true);
static SerialCommand c_disable("d", cmdDisable, true);
static SerialCommand c_fwd    ("f", cmdFwd,     true);
static SerialCommand c_rev    ("r", cmdRev,     true);
static SerialCommand c_brake  ("b", cmdBrake,   true);
static SerialCommand c_nobrake("n", cmdNoBrake, true);
static SerialCommand c_up     ("+", cmdUp,      true);
static SerialCommand c_down   ("-", cmdDown,    true);
static SerialCommand c_estop  ("x", cmdEstop,   true);
static SerialCommand c_help   ("?", cmdHelp,    true);
static SerialCommand c_zero   ("z", cmdZero,    true);
static SerialCommand c_magnet ("m", cmdMagnet,  true);
static SerialCommand c_set    ("s",     cmdSet);
static SerialCommand c_volts  ("v",     cmdVolts);
static SerialCommand c_sweep  ("sweep", cmdSweep);
static SerialCommand c_calsweep("calsweep", cmdCalSweep);
static SerialCommand c_calstep("calstep", cmdCalStep);
static SerialCommand c_scan   ("scan",  cmdScan);

// ── Setup / loop ─────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(200);

  // Control lines: open-drain by default (assert = pull to COM).
#if PUSH_PULL_CTRL
  pinMode(PIN_EN,  OUTPUT);
  pinMode(PIN_FR,  OUTPUT);
  pinMode(PIN_BRK, OUTPUT);
#else
  pinMode(PIN_EN,  OUTPUT_OPEN_DRAIN);
  pinMode(PIN_FR,  OUTPUT_OPEN_DRAIN);
  pinMode(PIN_BRK, OUTPUT_OPEN_DRAIN);
#endif
  pinMode(PIN_LED, OUTPUT);

  svBegin();
  svWrite(0);
  encBegin();

  // Boot safe: disabled, forward, no brake, zero speed.
  g_enabled = false; g_forward = true; g_brake = false;
  g_targetSv = g_sv = 0;
  applyOutputs();

#if HAS_SV_SENSE
  analogSetPinAttenuation(PIN_SV_SENSE, ADC_11db);  // full ~0-3.1V input range
#endif

  g_cli.SetDefaultHandler(cmdUnknown);
  g_cli.AddCommand(&c_enable);  g_cli.AddCommand(&c_disable);
  g_cli.AddCommand(&c_fwd);     g_cli.AddCommand(&c_rev);
  g_cli.AddCommand(&c_brake);   g_cli.AddCommand(&c_nobrake);
  g_cli.AddCommand(&c_up);      g_cli.AddCommand(&c_down);
  g_cli.AddCommand(&c_estop);   g_cli.AddCommand(&c_help);
  g_cli.AddCommand(&c_set);     g_cli.AddCommand(&c_volts);
  g_cli.AddCommand(&c_sweep);   g_cli.AddCommand(&c_scan);
  g_cli.AddCommand(&c_zero);    g_cli.AddCommand(&c_magnet);
  g_cli.AddCommand(&c_calsweep);
  g_cli.AddCommand(&c_calstep);

  Serial.println(F("\nBLD-120A bench test ready (boots DISABLED, command 0)."));
  Serial.printf("SV mode: %s on GPIO%d\n", SV_USE_DAC ? "DAC1 analog" : "LEDC PWM", PIN_SV);
  Serial.println(F("No PSU current limit on this bench -- protection is the driver's"));
  Serial.println(F("Peak Power trim + the software stall trip. Watch for the trip message."));
  printHelp();
}

void loop() {
  static uint32_t tTick = 0, tStatus = 0, tEnc = 0;
  g_cli.ReadSerial();

  const uint32_t now = millis();

  // Auto-sweep walks the target; the slew limiter below still smooths it.
  if (g_sweep && now >= g_sweepNext) {
    g_sweepNext = now + 100;
    g_targetSv += g_sweepDir * 16;
    if (g_targetSv >= CMD_MAX) { g_targetSv = CMD_MAX; g_sweepDir = -1; }
    else if (g_targetSv <= 0)  { g_targetSv = 0; g_sweep = false;
                                 Serial.println(F("> sweep complete")); }
    // Do NOT call g_stall.noteCommandIncrease() here. The grace window for a
    // sweep is opened exactly once, in cmdSweep() at sweep start — see the
    // comment there for why a per-step call in this loop is wrong (100 ms
    // step cadence vs. 300 ms grace re-extends the window forever and disables
    // the trip for nearly the whole ramp).
  }

  if (now - tTick >= TICK_MS) {
    tTick = now;

    // SV only lives while enabled and not braking; otherwise ramp to 0.
    const int want = (g_enabled && !g_brake) ? g_targetSv : 0;
    if (g_sv < want)      g_sv = min(g_sv + SLEW_STEP, want);
    else if (g_sv > want) g_sv = max(g_sv - SLEW_STEP, want);
    svWrite(g_sv);
    applyOutputs();
  }

  // Encoder sampling is DELIBERATELY faster than the 50 Hz control tick, and the
  // rate is a correctness constraint rather than a tuning knob. getCumulativePosition()
  // infers a wrap from a jump of more than half a turn (2048 counts), so the
  // measurable ceiling is 0.5 rev per sample. At the old 20 ms tick that capped
  // out at 1500 RPM — and bench-measured 1012 RPM/V means cmd 100 already sat at
  // 87 % of it. Past that, revolutions vanish silently: position stalls or walks
  // backwards while RPM reads plausibly low, which looks like a slipping magnet.
  // 4 ms lifts the ceiling to ~7500 RPM, clear of the ~3300 RPM this rig can
  // reach at the ESP's 3.3 V full command.
  if (g_encOk && now - tEnc >= ENC_TICK_MS) {
    tEnc = now;
    g_encPos = g_enc.getCumulativePosition();
  }

  if (g_encOk && now - g_fastTime >= RPM_FAST_MS) {
    const uint32_t dt = now - g_fastTime;
    if (g_fastTime && dt)
      g_rpmFast = (g_encPos - g_fastPos) * 60000.0f / ((float)ENC_CPR * dt);
    g_fastPos  = g_encPos;
    g_fastTime = now;

    // Armed whenever the driver is live. Not gated on any routine running —
    // manual `s`/`+` driving gets the same protection.
    if (g_enabled && !g_brake && g_stall.update(now, g_targetSv, g_rpmFast)) {
      Serial.printf("\n*** STALL TRIP at cmd %d — commanded but not turning ***\n",
                    g_stall.trippedAtCmd());
      Serial.println(F("*** SV cut, brake asserted. Check for a jam before retrying. ***"));
      estop();
    }
  }

  // Non-blocking by construction: every dwell is a millis() comparison, never a
  // delay(). A blocking sweep would stop g_cli.ReadSerial() from running and
  // leave `x` dead for the full 158 s.
  if (g_cal == CalState::Settle && now >= g_calUntil) {
    g_calPos0 = g_encPos;
    g_calT0   = now;
    g_cal     = CalState::Measure;
    g_calUntil = now + CAL_MEASURE_MS;
  } else if (g_cal == CalState::Measure && now >= g_calUntil) {
    const uint32_t dt     = now - g_calT0;
    const int32_t  dcount = g_encPos - g_calPos0;
    const float    rpm    = dt ? dcount * 60000.0f / ((float)ENC_CPR * dt) : 0.0f;
    const int      cmd    = calCmdForLeg(g_calLeg, g_calIdx);
    const uint8_t  status = g_enc.readStatus();

    Serial.printf("CSV,sweep,%s,%s,%d,%.3f,%.1f,%.1f,%ld,%u,0x%02X\n",
                  calLegIsHigh(g_calLeg) ? "HIGH" : "LOW",
                  calLegIsUp(g_calLeg) ? "up" : "down",
                  cmd, ESP_VMAX * cmd / (float)CMD_MAX,
                  rpm, rpm / GEAR_RATIO, (long)dcount,
                  g_enc.readAGC(), status);

    // Supply-sag / driver current-limiting: on an ASCENDING leg, more command
    // must not produce less speed. When it does, the supply is sagging or the
    // driver is limiting internally — and the resulting bend looks exactly like
    // a genuine saturation knee in the fitted curve. Flag it at the point it
    // happens so the analysis is not silently reading a power problem as motor
    // physics. Threshold is 2% to clear ordinary point-to-point noise.
    if (calLegIsUp(g_calLeg) && g_calIdx > 0 &&
        fabsf(g_calPrevRpm) > 50.0f &&
        fabsf(rpm) < fabsf(g_calPrevRpm) * 0.98f) {
      Serial.printf("! SAG at cmd %d: %.1f -> %.1f RPM while command ROSE"
                    " (supply sagging or driver limiting — not a real knee)\n",
                    cmd, g_calPrevRpm, rpm);
    }
    g_calPrevRpm = rpm;

    // A weak magnet fails intermittently. Logging status per point turns a
    // silent mid-leg dropout into a visible abort instead of a plausible curve
    // with a hole in it.
    if (g_enc.magnetTooWeak() || !g_enc.magnetDetected()) {
      Serial.printf("! MAGNET FAULT at cmd %d — CALSWEEP ABORTED (entire run, all"
                    " remaining legs skipped), data unusable\n", cmd);
      estop();
    } else if (++g_calIdx >= CAL_POINTS) {
      g_calIdx = 0;
      if (++g_calLeg >= 4) {
        g_cal = CalState::Done;
        setCommand(0, "calsweep complete");
        g_enabled = false;
        Serial.println(F("> CALSWEEP complete."));
      } else {
        g_forward = calLegIsHigh(g_calLeg);
        // reset() BEFORE setCommand(): reset() zeroes grace_until_ back to `now`,
        // so calling it after setCommand() would erase the grace window
        // setCommand()'s noteCommandIncrease() just opened — same ordering as
        // cmdCalSweep()'s own setup, above.
        g_stall.reset(now);          // direction change: restart trip timing
        setCommand(calCmdForLeg(g_calLeg, 0), "calsweep leg");
        g_calPrevRpm = 0.0f;         // sag comparison must not cross legs
        g_cal = CalState::Settle;
        g_calUntil = now + CAL_SETTLE_MS;
      }
    } else {
      setCommand(calCmdForLeg(g_calLeg, g_calIdx), "calsweep");
      g_cal = CalState::Settle;
      g_calUntil = now + CAL_SETTLE_MS;
    }
  }

  // calstep: rise -> settle -> coast (power off, brake off) -> brake (power on,
  // then brake) -> done. Coast and Brake deliberately stop the motor while
  // g_targetSv is still nonzero — see the comments at each transition below for
  // how that interacts with g_stall, which must stay armed the whole time.
  if (g_step != StepPhase::Idle && g_step != StepPhase::Done && now >= g_stepNext) {
    g_stepNext = now + STEP_LOG_MS;
    const char* phase = (g_step == StepPhase::Rise)  ? "rise"
                      : (g_step == StepPhase::Coast) ? "coast"
                      : (g_step == StepPhase::Brake) ? "brake" : "settle";
    if (g_step != StepPhase::Settle)
      Serial.printf("CSV,step,%s,%lu,%d,%.1f\n",
                    phase, (unsigned long)(now - g_stepT0), g_targetSv, g_rpmFast);

    const bool stopped = fabsf(g_rpmFast) < STEP_STOP_RPM;
    switch (g_step) {
      case StepPhase::Rise:
        if (now - g_stepT0 >= STEP_RISE_MS) {
          g_step = StepPhase::Settle; g_stepT0 = now;   // hold before coasting
        }
        break;
      case StepPhase::Settle:
        if (now - g_stepT0 >= 500) {
          // Coast = disabled with the brake OFF: pure mechanical friction.
          // g_enabled goes false here, so the stall-trip gate below
          // (`g_enabled && !g_brake && g_stall.update(...)`) short-circuits
          // for the whole coast: g_stall.update() is simply never called
          // while g_targetSv sits at g_stepCmd and the motor spins down on
          // its own. That is the mechanism that keeps a deliberate coast from
          // ever being mistaken for a stall — not a grace window, but the
          // trip not running at all while the driver is disabled.
          g_enabled = false; g_brake = false;
          g_step = StepPhase::Coast; g_stepT0 = now;
        }
        break;
      case StepPhase::Coast:
        if (stopped || now - g_stepT0 >= STEP_STOP_MS) {
          // Spin back up, then brake, so coast and brake are compared from the
          // same starting speed.
          g_enabled = true; g_brake = false;
          setCommand(g_stepCmd, "calstep respin");
          g_stall.reset(now);
          // setCommand() above will NOT have opened a grace window: g_targetSv
          // has held at g_stepCmd since Rise (nothing zeroed it during Settle
          // or Coast), so setCommand()'s internal "did it increase?" check
          // sees no change and skips noteCommandIncrease(). Left alone, reset()
          // just above leaves grace_until_ == now (zero grace) at the exact
          // moment the motor is commanded from a dead stop — the textbook
          // false-trip case update() would otherwise flag on the very next
          // 100 ms sample. Call it explicitly, same fix as cmdEnable()/
          // cmdNoBrake() already apply to their own resume paths.
          g_stall.noteCommandIncrease(now);
          g_step = StepPhase::Brake; g_stepT0 = now;
        }
        break;
      case StepPhase::Brake:
        // Give it STEP_RISE_MS to reach speed again, then slam the brake.
        if (now - g_stepT0 >= STEP_RISE_MS && !g_brake) g_brake = true;
        // Once g_brake is true, the stall-trip gate's `!g_brake` term is
        // false, so — same short-circuit as the Coast phase above —
        // g_stall.update() is never called while the brake is actually
        // decelerating the motor. Before that point (the respin leg), the
        // grace window opened above covers the spin-up.
        if (g_brake && (stopped || now - g_stepT0 >= STEP_RISE_MS + STEP_STOP_MS)) {
          setCommand(0, "calstep complete");
          g_enabled = false;
          g_step = StepPhase::Done;
          Serial.println(F("> CALSTEP complete."));
        }
        break;
      default: break;
    }
  }

  if (now - tStatus >= STATUS_MS) {
    // Derive RPM over the whole status window rather than per tick: 500 ms of
    // counts gives ~0.03 RPM resolution, where a single-tick delta would quantise
    // to ~0.7 RPM and jitter badly at low speed.
    if (g_encOk && g_rpmTime) {
      const uint32_t dt = now - g_rpmTime;
      if (dt) g_rpm = (g_encPos - g_rpmPos) * 60000.0f / ((float)ENC_CPR * dt);
    }
    g_rpmPos  = g_encPos;
    g_rpmTime = now;

    tStatus = now;
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));  // heartbeat
    printStatus();
  }
}
