// ONE-OFF bring-up test: the RL corner on the fabricated Wheel Drive PCB.
//
// WHY THIS FILE EXISTS, AND WHY IT DOES NOT REUSE bench_bld_drive.h
// -----------------------------------------------------------------
// The breadboard pin table in src/bench_bld_drive.h and the netlist-derived map
// in include/pins.h DISAGREE for the FR and RL corners — the four roles are in
// opposite order:
//
//   bench_bld_drive.h  RL:  BRK=18  EN=17  F/R=16  SV=15
//   include/pins.h     RL:  SV =18  F/R=17  EN =16  BRK=15
//
// Flashing the bench rig against this PCB would send the speed PWM to GPIO15,
// which the board wires to the driver's BRAKE input, and read GPIO18 (the real
// SV) as a brake output. The motor would sit still with no fault light and no
// obvious cause.
//
// So this file takes its pins from pins.h and nothing else. It prints them at
// boot so they can be checked against the schematic before anything spins.
//
// SCOPE: RL only. FL/FR/RR are left untouched and un-configured — their GPIOs
// stay Hi-Z, which the transistor adapters read as "released" = disabled.
//
// BUILD:  pio run -e pcb_rl -t upload && pio device monitor -e pcb_rl
#include <Arduino.h>

#include "pins.h"

// ---------------------------------------------------------------------------
// Pins — RL is index 2 of every array in pins.h. Read, never retyped.
// ---------------------------------------------------------------------------
static constexpr uint8_t kCorner = 2;  // FL=0, FR=1, RL=2, RR=3

static constexpr uint8_t PIN_SV  = pins::kWheelSV[kCorner];
static constexpr uint8_t PIN_FR  = pins::kWheelFR[kCorner];
static constexpr uint8_t PIN_EN  = pins::kWheelEN[kCorner];
static constexpr uint8_t PIN_BRK = pins::kWheelBRK[kCorner];

// Catch a future pins.h edit that collides two roles onto one GPIO, at compile
// time rather than by watching a motor misbehave.
static_assert(PIN_SV != PIN_FR && PIN_SV != PIN_EN && PIN_SV != PIN_BRK &&
                  PIN_FR != PIN_EN && PIN_FR != PIN_BRK && PIN_EN != PIN_BRK,
              "RL pins collide in pins.h");

// ---------------------------------------------------------------------------
// Polarity — transistor adapters ARE populated on this board.
//
// The NPN inverts: a HIGH on the GPIO turns it on, which pulls the driver input
// down to COM = asserted. The output latch resets LOW, so every line boots
// released and the motor boots disabled. The safe state is free.
// ---------------------------------------------------------------------------
static constexpr uint8_t ASSERT_LVL  = HIGH;
static constexpr uint8_t RELEASE_LVL = LOW;
static constexpr uint8_t FWD_LVL     = LOW;   // released = driver sees HIGH = forward
static constexpr uint8_t REV_LVL     = HIGH;

static constexpr uint32_t PWM_HZ   = pins::kSvPwmFreqHz;   // 2 kHz — inside both manuals
static constexpr uint8_t  PWM_BITS = pins::kPwmResBits;    // 12-bit
static constexpr uint16_t PWM_MAX  = (1u << PWM_BITS) - 1u;

// Idle auto-stop. This is a real drivetrain on a bench, not an LED: if the
// console goes quiet while the motor is spinning — terminal closed, USB pulled,
// attention elsewhere — it must not keep running. Same reasoning as LinkWatchdog,
// scaled down to one wheel.
static constexpr uint32_t IDLE_STOP_MS = 8000;

// ---------------------------------------------------------------------------
// FULL-BOARD SIGNAL SCAN — all 16 wheel control lines, one pin at a time.
//
// This tests the BOARD, not the driver: no motor, no 24 V, no wheels needed.
// The ESP senses its own pins, so it can find the manufacturing defects that a
// motion test cannot distinguish from a dead driver channel:
//
//   stuck LOW   — trace shorted to ground, or a solder bridge to a GND pour
//   stuck HIGH  — shorted to 3V3
//   bridged     — two signal traces shorted to each other
//   open        — nothing detectable from here; that needs the meter walk ('w')
//
// HOW THE READINGS WORK
// Each pin is pulled up, then pulled down, and read back both times:
//     up=1 down=0  -> pin moves freely, as a good trace should
//     up=0 down=0  -> held low by something
//     up=1 down=1  -> held high by something
//
// IMPORTANT CAVEAT: a populated transistor adapter LOADS the pin. Its ~100k
// base pulldown against the ESP's ~45k internal pull-up divides to roughly 2.3 V,
// which is below the S3's input-high threshold — so a perfectly good adapter
// channel can read "up=0" and look stuck low. That is why the verdicts below are
// ADVISORY and why the host script compares against a known-good golden board
// instead of trusting absolute values. Same board revision, same populated
// parts, same expected signature.
// ---------------------------------------------------------------------------
static constexpr uint8_t kScanGpio[16] = {
    pins::kWheelSV[0], pins::kWheelFR[0], pins::kWheelEN[0], pins::kWheelBRK[0],
    pins::kWheelSV[1], pins::kWheelFR[1], pins::kWheelEN[1], pins::kWheelBRK[1],
    pins::kWheelSV[2], pins::kWheelFR[2], pins::kWheelEN[2], pins::kWheelBRK[2],
    pins::kWheelSV[3], pins::kWheelFR[3], pins::kWheelEN[3], pins::kWheelBRK[3],
};
static const char* const kCornerName[4] = {"FL", "FR", "RL", "RR"};
static const char* const kRoleName[4]   = {"SV ", "F/R", "EN ", "BRK"};
static constexpr uint8_t kScanN = 16;

static void scanName(uint8_t i, char* out) {
  snprintf(out, 8, "%s.%s", kCornerName[i / 4], kRoleName[i % 4]);
}

// Park every scan pin as a plain high-impedance input. Nothing is driven, so
// nothing can be asserted at the driver while we measure.
static void scanIdle() {
  for (uint8_t i = 0; i < kScanN; ++i) pinMode(kScanGpio[i], INPUT);
}

static void scanAll() {
  Serial.println(F("\n=== SIGNAL SCAN: 16 wheel control lines ==="));
  Serial.println(F("MOTOR POWER MUST BE OFF. Pins are driven individually."));
  Serial.println(F("name     gpio  up dn  verdict        bridged-to"));

  scanIdle();
  delay(5);

  char nm[8];
  uint8_t faults = 0;

  for (uint8_t i = 0; i < kScanN; ++i) {
    scanName(i, nm);

    pinMode(kScanGpio[i], INPUT_PULLUP);
    delay(3);
    const int up = digitalRead(kScanGpio[i]);
    pinMode(kScanGpio[i], INPUT_PULLDOWN);
    delay(3);
    const int dn = digitalRead(kScanGpio[i]);
    pinMode(kScanGpio[i], INPUT);

    // "held low" is EXPECTED on any line with a populated adapter -- the base
    // pulldown wins against the internal pull-up. It is only a fault if it
    // differs from the golden board, which is the host script's job to decide.
    // Counting it here would report a healthy board as 3 faults every time.
    const char* verdict = "ok/floating ";
    if (up == 0 && dn == 0) verdict = "low (loaded?)";
    else if (up == 1 && dn == 1) { verdict = "HELD HIGH   "; ++faults; }

    // Bridge hunt. A neighbour counts as bridged only if it TRACKS the driven
    // pin -- high when we drive high AND low when we drive low.
    //
    // Requiring both passes is the whole trick. An earlier version accepted a
    // match on either pass, which meant any stuck-low pin "matched" every time
    // we drove something low and got reported as bridged to all 15 others. On a
    // board with three adapter-loaded lines that produced 45 phantom bridges and
    // would have buried a genuine one completely.
    bool tracks_high[kScanN] = {false};
    char bridges[96] = "";
    for (uint8_t pass = 0; pass < 2; ++pass) {
      const int level = (pass == 0) ? HIGH : LOW;
      for (uint8_t j = 0; j < kScanN; ++j)
        if (j != i) pinMode(kScanGpio[j], (pass == 0) ? INPUT_PULLDOWN : INPUT_PULLUP);
      pinMode(kScanGpio[i], OUTPUT);
      digitalWrite(kScanGpio[i], level);
      delay(3);
      for (uint8_t j = 0; j < kScanN; ++j) {
        if (j == i) continue;
        const bool follows = (digitalRead(kScanGpio[j]) == level);
        if (pass == 0) {
          tracks_high[j] = follows;
        } else if (follows && tracks_high[j]) {
          char other[8];
          scanName(j, other);
          strncat(bridges, other, sizeof(bridges) - strlen(bridges) - 2);
          strncat(bridges, " ", sizeof(bridges) - strlen(bridges) - 2);
        }
      }
      pinMode(kScanGpio[i], INPUT);
    }
    scanIdle();

    if (bridges[0]) ++faults;
    Serial.printf("%-8s %-4u  %d  %d  %s %s\n", nm, kScanGpio[i], up, dn, verdict, bridges);
  }

  scanIdle();
  Serial.printf("=== SCAN END: %u anomaly(ies) ===\n", faults);
  Serial.println(F("Compare against a known-good board before believing any of it."));

  // Leave RL back under normal control so 't' still works afterwards.
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_BRK, OUTPUT);
  pinMode(PIN_FR, OUTPUT);
  digitalWrite(PIN_EN, RELEASE_LVL);
  digitalWrite(PIN_BRK, RELEASE_LVL);
  digitalWrite(PIN_FR, FWD_LVL);
  ledcAttach(PIN_SV, PWM_HZ, PWM_BITS);
  ledcWrite(PIN_SV, 0);
}

// Meter walk: assert one line at a time and hold it, so the pin can be probed at
// the header. This is the only way to catch an OPEN trace — the ESP cannot sense
// a wire that goes nowhere, it can only sense shorts.
static void walkPins() {
  Serial.println(F("\n=== METER WALK — probe each header pin against COM ==="));
  Serial.println(F("MOTOR POWER OFF. Each line is driven HIGH for 4 s in turn."));
  Serial.println(F("Expect ~3.3 V at the ESP pad and a switched level at the header."));
  Serial.println(F("Any key aborts.\n"));
  scanIdle();

  char nm[8];
  for (uint8_t i = 0; i < kScanN; ++i) {
    scanName(i, nm);
    pinMode(kScanGpio[i], OUTPUT);
    digitalWrite(kScanGpio[i], HIGH);
    Serial.printf("  %-8s GPIO%-3u driven HIGH ...\n", nm, kScanGpio[i]);
    const uint32_t t0 = millis();
    while (millis() - t0 < 4000) {
      if (Serial.available()) {
        while (Serial.available()) Serial.read();
        digitalWrite(kScanGpio[i], LOW);
        pinMode(kScanGpio[i], INPUT);
        scanIdle();
        Serial.println(F(">> walk aborted"));
        return;
      }
      delay(10);
    }
    digitalWrite(kScanGpio[i], LOW);
    pinMode(kScanGpio[i], INPUT);
  }
  scanIdle();
  Serial.println(F("=== WALK END ==="));
}

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
static bool     g_enabled  = false;
static bool     g_braked   = false;
static bool     g_reverse  = false;
static uint8_t  g_duty_pct = 0;
static uint32_t g_last_cmd_ms = 0;

static void applyOutputs() {
  digitalWrite(PIN_FR, g_reverse ? REV_LVL : FWD_LVL);
  digitalWrite(PIN_BRK, g_braked ? ASSERT_LVL : RELEASE_LVL);
  digitalWrite(PIN_EN, g_enabled ? ASSERT_LVL : RELEASE_LVL);
  const uint32_t duty = (uint32_t)PWM_MAX * g_duty_pct / 100u;
  ledcWrite(PIN_SV, duty);
}

static void stopAll(const char* why) {
  g_duty_pct = 0;
  g_enabled = false;
  g_braked = false;
  applyOutputs();
  Serial.printf("\n>> STOP (%s)\n", why);
}

static void printState() {
  Serial.printf("  EN=%s  BRK=%s  DIR=%s  DUTY=%u%%  (SV raw %u/%u)\n",
                g_enabled ? "ASSERTED" : "released",
                g_braked ? "ASSERTED" : "released",
                g_reverse ? "REVERSE" : "forward",
                g_duty_pct, (uint32_t)PWM_MAX * g_duty_pct / 100u, PWM_MAX);
}

static void printHelp() {
  Serial.println(F(
      "\n  e  toggle ENABLE      SPACE  stop everything\n"
      "  + / -  duty +/-5%     0  duty to zero\n"
      "  d  direction flip     b  toggle BRAKE\n"
      "  t  automatic self-test (ramps to 20%, both directions)\n"
      "  p  print state        ?  this help\n"
      "\n  BOARD TESTS — motor power OFF, no motor needed:\n"
      "  s  signal scan: all 16 wheel lines, shorts and bridges\n"
      "  w  meter walk: drive each line 4 s so you can probe it\n"));
}

// A hands-off sequence that proves the whole chain without needing fast typing.
static void selfTest() {
  Serial.println(F("\n=== SELF-TEST: RL — watch the wheel ==="));
  g_braked = false;
  g_reverse = false;
  g_enabled = true;
  g_duty_pct = 0;
  applyOutputs();
  Serial.println(F("EN asserted, duty 0. Wheel should be free but energised."));
  delay(1000);

  for (int dir = 0; dir < 2; ++dir) {
    g_reverse = (dir == 1);
    Serial.printf("\n-- %s --\n", g_reverse ? "REVERSE" : "FORWARD");
    for (uint8_t d = 5; d <= 20; d += 5) {
      g_duty_pct = d;
      applyOutputs();
      Serial.printf("   duty %2u%%\n", d);
      delay(1500);
      // Any keypress aborts — the operator is the emergency stop here.
      if (Serial.available()) {
        while (Serial.available()) Serial.read();
        stopAll("aborted by keypress");
        return;
      }
    }
    g_duty_pct = 0;
    applyOutputs();
    delay(1200);
  }
  stopAll("self-test complete");
  Serial.println(F("If the wheel turned BOTH ways, the RL channel is good."));
}

void setup() {
  Serial.begin(115200);

  // *** DO NOT REMOVE. This is what stops the sketch hanging between boards. ***
  //
  // With ARDUINO_USB_MODE=1, Serial is HWCDC, and HWCDC BLOCKS when its TX
  // buffer fills with no host draining it. The heartbeat below prints once a
  // second, so the instant the test script closes the port, this sketch fills
  // the buffer and blocks inside Serial.printf -- forever. The next board then
  // finds a chip that never answers, and the failure looks exactly like dead
  // firmware or a dead board. It cost an evening on 2026-08-06.
  //
  // Timeout 0 = never block; discard instead. Losing output to a terminal that
  // is not attached is the correct trade against wedging the drivetrain
  // controller.
  Serial.setTxTimeoutMs(0);

  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  // pinMode FIRST. On Arduino-ESP32 3.x a digitalWrite to a pin that is not yet
  // configured is REJECTED and logged, so the old "set the level before the
  // pinMode to close the boot window" trick silently does nothing.
  //
  // What actually protects us is the adapter polarity: the output latch resets
  // to 0 = LOW, and with the inverting NPN LOW = RELEASED = driver disabled. The
  // safe state is the power-on default, so there is no window to close. That is
  // a property of the transistor stage, not of the write ordering — if these
  // lines ever go back to driving the driver directly (LOW = asserted), boot
  // safety has to be reasoned about again from scratch.
  pinMode(PIN_EN, OUTPUT);
  pinMode(PIN_BRK, OUTPUT);
  pinMode(PIN_FR, OUTPUT);
  digitalWrite(PIN_EN, RELEASE_LVL);
  digitalWrite(PIN_BRK, RELEASE_LVL);
  digitalWrite(PIN_FR, FWD_LVL);

  ledcAttach(PIN_SV, PWM_HZ, PWM_BITS);
  ledcWrite(PIN_SV, 0);

  Serial.println(F("\n\n================ RL CORNER BRING-UP (Wheel Drive PCB) ================"));
  Serial.println(F("Pins read from include/pins.h — check these against the schematic:"));
  Serial.printf("    SV  (speed) = GPIO%-3u    F/R (dir)   = GPIO%u\n", PIN_SV, PIN_FR);
  Serial.printf("    EN  (enable)= GPIO%-3u    BRK (brake) = GPIO%u\n", PIN_EN, PIN_BRK);
  Serial.printf("    PWM %u Hz, %u-bit.  Adapters: NPN inverting (HIGH = asserted).\n",
                (unsigned)PWM_HZ, (unsigned)PWM_BITS);
  Serial.println(F("Only RL is configured. FL/FR/RR stay Hi-Z = disabled."));
  Serial.printf("Auto-stop after %u s with no keypress while spinning.\n", IDLE_STOP_MS / 1000);
  printHelp();
  Serial.println(F("Wheels OFF THE GROUND. Press 't' for the self-test.\n"));

  g_last_cmd_ms = millis();
}

void loop() {
  // Heartbeat until the first keypress. The boot banner prints once and is lost
  // if no terminal is attached yet (USB-CDC discards TX with no host), which is
  // indistinguishable from "the sketch never ran". This makes attaching late
  // still show proof of life.
  // Belt and braces alongside setTxTimeoutMs(0): give up after 30 s regardless.
  // A heartbeat exists to prove life to someone who just attached, not to
  // narrate into an empty room for hours.
  static uint32_t last_beat = 0;
  static bool seen_input = false;
  if (!seen_input && millis() > 30000) seen_input = true;
  if (!seen_input && millis() - last_beat >= 1000) {
    last_beat = millis();
    Serial.printf("[alive %lus] RL: SV=GPIO%u F/R=GPIO%u EN=GPIO%u BRK=GPIO%u — press ? for help\n",
                  (unsigned long)(millis() / 1000), PIN_SV, PIN_FR, PIN_EN, PIN_BRK);
  }

  if (Serial.available()) {
    seen_input = true;
    const int c = Serial.read();
    g_last_cmd_ms = millis();
    switch (c) {
      case 'e': g_enabled = !g_enabled; applyOutputs(); printState(); break;
      case 'b': g_braked = !g_braked; applyOutputs(); printState(); break;
      case 'd': g_reverse = !g_reverse; applyOutputs(); printState(); break;
      case '+': case '=':
        if (g_duty_pct <= 95) g_duty_pct += 5;
        applyOutputs(); printState(); break;
      case '-': case '_':
        if (g_duty_pct >= 5) g_duty_pct -= 5;
        applyOutputs(); printState(); break;
      case '0': g_duty_pct = 0; applyOutputs(); printState(); break;
      case ' ': stopAll("operator"); break;
      case 't': selfTest(); break;
      // Scans drive lines individually. Refuse while the drivetrain is live —
      // driving SV HIGH is a 100% speed command, and doing that with EN already
      // asserted would put the wheel straight to full pelt.
      case 's':
        if (g_enabled || g_duty_pct) { Serial.println(F("stop first (space)")); break; }
        scanAll(); break;
      case 'w':
        if (g_enabled || g_duty_pct) { Serial.println(F("stop first (space)")); break; }
        walkPins(); break;
      case 'p': printState(); break;
      case '?': case 'h': printHelp(); break;
      default: break;   // ignore newlines and stray bytes
    }
  }

  // Idle auto-stop: only meaningful while something could actually be moving.
  if (g_duty_pct > 0 && g_enabled && millis() - g_last_cmd_ms >= IDLE_STOP_MS) {
    stopAll("idle timeout — no keypress");
    g_last_cmd_ms = millis();
  }
}
