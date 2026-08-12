// Bluepad32 gamepad drive for the FABRICATED Wheel Drive PCB.
//
// WHY THIS EXISTS RATHER THAN REUSING sketch_bld.cpp
// --------------------------------------------------
// sketch_bld.cpp drives through bench_bld_drive.h, whose pin table was measured
// on the BREADBOARD. On the fabricated board two corners have their roles
// exactly swapped:
//
//     GPIO21   bench: FR brake        pins.h: FR speed PWM
//     GPIO38   bench: FR speed PWM    pins.h: FR brake
//     GPIO15   bench: RL speed PWM    pins.h: RL brake
//     GPIO18   bench: RL brake        pins.h: RL speed PWM
//
// Flashing it here would send speed PWM into two brake lines and hold two speed
// inputs as enables. This file reads include/pins.h -- the netlist-derived map --
// and never a hand-typed table.
//
// SIMULATED ALIGNMENT
// -------------------
// The ToF mounts are not built yet, so 'aligned' cannot be measured. Y on the
// gamepad (or 'a' on serial) toggles a SIMULATED alignment flag so the indicator
// and anything downstream of it can be exercised now. It is deliberately loud
// about being fake: nothing should ever ship reading this flag.
//
// CONTROLS
//   DRIVE   left stick   move / strafe      right stick X  rotate
//           LB / RB      slower / faster    B              STOP EVERYTHING
//   ARM     D-pad U/D    jog X (held)       D-pad L/R      jog Y (held)
//           A            clamp sequence     X              release / stow
//           LT           flip X             RT             flip Y
//   Y       toggle SIMULATED aligned
//
// SAFETY
//   * no gamepad at boot -> motors stay disabled; nothing spins on power-up
//   * gamepad disconnect -> wheels stop AND any latched arm jog is stopped
//   * B sends the arm's E-STOP as well as stopping the wheels: one control the
//     operator can reach without first working out which subsystem is at fault
//   * drive speed is capped at 40% -- there is no stall detection on this base
//   WHEELS OFF THE GROUND.
#include <Arduino.h>
#include <Bluepad32.h>

#include "bench_mix.h"   // pure mecanum mix, shared with the bench rigs
#include "config.h"
#include "pins.h"        // NETLIST-DERIVED. never hand-type a pin table.

// The REAL alignment stack, reused rather than reimplemented. These are the
// same classes main.cpp runs and the same ones covered by the 70 host tests --
// a second copy of a docking state machine is the last thing this project needs.
#include "ArduinoClock.h"
#include "CornerEdgeDetector.h"
#include "DeadReckonOdometry.h"
#include "DockingStateMachine.h"
#include "SafetyMonitor.h"
#include "Vl53l0xArray.h"

using namespace tb;

// Adapter polarity. The NPN stage INVERTS: HIGH at the GPIO asserts the driver
// input. The output latch resets LOW, so every line boots released and the
// motors boot disabled -- the safe state is the power-on default.
static constexpr uint8_t ASSERT_LVL = HIGH, RELEASE_LVL = LOW;
static constexpr uint8_t FWD_LVL = LOW, REV_LVL = HIGH;
static constexpr uint32_t PWM_HZ = 2000;      // inside BOTH driver manuals' ranges
static constexpr uint8_t PWM_BITS = 12;
static constexpr uint16_t PWM_MAX = (1u << PWM_BITS) - 1u;

static const char* WHEEL[4] = {"FL", "FR", "RL", "RR"};

// PER-WHEEL DIRECTION CALIBRATION.
//
// VERIFIED ON THE ASSEMBLED BASE 2026-08-11: with an empty table both LEFT
// wheels ran backwards, so FL and RL need their F/R line flipped for a forward
// command to drive forward. The left-hand motors are mounted mirrored to the
// right-hand pair -- a mechanical fact about the chassis, not a wiring error.
//
// This is NOT inherited from the breadboard rig's table (which happened to hold
// the same values): bench "FL" is this board's RR and its two middle corners
// were driving the wrong lines entirely, so that calibration described nothing.
// This one was re-derived here, on this hardware.
static bool g_invert[4] = {true, false, true, false};   // FL FR RL RR

static bool g_rev[4] = {false, false, false, false};
static uint16_t g_duty[4] = {0, 0, 0, 0};

static ControllerPtr g_ctl = nullptr;
static bool g_pairing = false;
static uint8_t g_cal_sel = 0;
static bool g_sim_aligned = false;

// SPEED LIMITS -- CEILING IS 40%, ON PURPOSE.
// There is no encoder feedback and no stall detection on this base (no ALARM
// terminal, encoders deferred), so nothing in software can tell a jammed wheel
// from a loaded one. 40% of a drivetrain that reaches ~1400 mm/s is plenty for
// manual work and leaves the operator time to react.
static const float LIMITS[] = {0.10f, 0.20f, 0.30f, 0.40f};
static uint8_t g_limit_idx = 0;

// ---- link to ESP-ARM ------------------------------------------------------
//
// NOT GPIO43/44. Those are UART0, which this firmware's console already uses
// (CONFIG_ESP_CONSOLE_UART_NUM=0), so driving them would collide with the
// operator console. GPIO1/2 are free -- they were the analog encoder fallback
// in pins.h, unused since the encoders moved to I2C.
//
// The ARM side is stock: its Serial is UART0 at 115200 on ITS GPIO43/44, and it
// parses single characters, so it needs no firmware change to accept these.
//
//   base GPIO1 (TX) --> arm GPIO44 (RX)
//   base GPIO2 (RX) <-- arm GPIO43 (TX)
//   GND <-> GND
//
// CAUTION: the arm's UART0 is shared with its USB-UART bridge. Do not leave the
// arm's USB cable plugged into a computer while this link is driving it, or two
// transmitters fight over the arm's RX line.
static constexpr uint8_t ARM_TX = 1, ARM_RX = 2;
static bool g_arm_passthrough = false;

// ------------------------------------------------------------------ drive

static void pcbInit() {
  for (uint8_t i = 0; i < 4; ++i) {
    // pinMode FIRST. Arduino-ESP32 3.x REJECTS a digitalWrite to a pin that is
    // not yet configured -- the old "write before pinMode to close the boot
    // window" trick silently does nothing. Boot safety comes from the adapter
    // polarity instead: latch resets LOW = released = disabled.
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

static void pcbStopAll() {
  for (uint8_t i = 0; i < 4; ++i) {
    ledcWrite(pins::kWheelSV[i], 0);
    g_duty[i] = 0;
    digitalWrite(pins::kWheelEN[i], RELEASE_LVL);
    digitalWrite(pins::kWheelBRK[i], RELEASE_LVL);
  }
}

// Signed command in [-1, 1]. Zero RELEASES enable (coast) rather than braking:
// braking every time the stick centres would be violent and would fight the
// driver's own ramp.
static void pcbWheel(uint8_t i, float v) {
  if (g_invert[i]) v = -v;
  v = constrain(v, -1.0f, 1.0f);

  const bool rev = v < 0.0f;
  const float mag = fabsf(v);

  // Reversing under power is hard on the driver -- the manual says stop first --
  // so collapse duty in the same write that flips the direction line.
  if (rev != g_rev[i]) {
    ledcWrite(pins::kWheelSV[i], 0);
    g_duty[i] = 0;
    g_rev[i] = rev;
    digitalWrite(pins::kWheelFR[i], rev ? REV_LVL : FWD_LVL);
  }

  const uint16_t duty = (uint16_t)lroundf(mag * PWM_MAX);
  digitalWrite(pins::kWheelEN[i], duty > 0 ? ASSERT_LVL : RELEASE_LVL);
  digitalWrite(pins::kWheelBRK[i], RELEASE_LVL);
  ledcWrite(pins::kWheelSV[i], duty);
  g_duty[i] = duty;
}

static void pcbDriveMix(float vx, float vy, float w, float limit) {
  float o[4];
  mixWheels(vx, vy, w, o);
  // The limit scales all four uniformly -- scaling one alone would change the
  // direction of travel, not just the speed.
  for (uint8_t i = 0; i < 4; ++i) pcbWheel(i, o[i] * limit);
}

// ------------------------------------------------------ domain adapters
//
// Thin shims so DockingStateMachine can drive THIS sketch's motor layer.

class GamepadDrive : public IDrive {
 public:
  void move(const DriveCommand& c) override {
    // limit 1.0: the state machine's own config already sets its speeds, and
    // scaling them again by the operator's drive limit would silently change
    // the docking behaviour whenever someone touched a bumper.
    pcbDriveMix(c.vx, c.vy, c.omega, 1.0f);
  }
  void stop() override { pcbStopAll(); }
  void enable(bool) override {}          // per-wheel EN is handled in pcbWheel
  void brake(bool on) override {
    for (uint8_t i = 0; i < 4; ++i)
      digitalWrite(pins::kWheelBRK[i], on ? ASSERT_LVL : RELEASE_LVL);
  }
  bool fault() const override { return false; }   // no ALARM net on this board
};

// The clamp is not on this board -- it is the ARM, reached over the serial
// link. These no-op because the base sends 'b' itself on confirmation; letting
// the state machine run its own ClampEngage would mean waiting on limit
// switches it cannot see (they are on the arm, which reports only text).
class NullClamp : public IClamp {
 public:
  void open(float) override {}
  void close(float) override {}
  void stop() override {}
  void enable(bool) override {}
  float currentAmps() const override { return 0.0f; }
};

class NullLimits : public ILimitSwitches {
 public:
  void update() override {}
  bool clampOpen() const override { return false; }
  bool clampClosed() const override { return false; }
};

static GamepadDrive  g_drive;
static NullClamp     g_clamp;
static NullLimits    g_limits;
static ArduinoClock  g_clock;
static Vl53l0xArray  g_tof(pins::kTofXSHUT, cfg::kNumZones);
static CornerEdgeDetector g_edge(cfg::makeCornerConfig());
static DeadReckonOdometry g_odom(cfg::makeOdometryCal());
static SafetyMonitor g_safety;
static DockingStateMachine g_sm(g_drive, g_clamp, g_limits, g_odom, g_safety,
                                g_clock, cfg::makeDockConfig());
static bool g_docking = false;
static bool g_tof_ok = false;

// ------------------------------------------------------------- arm link

static void armSend(char c, const char* what) {
  Serial1.write((uint8_t)c);
  Console.printf("arm <- '%c'  (%s)\n", c, what);
}

// JOG IS LATCHED ON THE ARM SIDE.
//
// The arm's 'e'/'r'/'E'/'R' run until an end-stop or an E-STOP -- there is no
// "while held" in its serial handler. A bare button press would therefore start
// motion that does not stop when the operator lets go, which is not what a
// D-pad feels like it should do.
//
// So we synthesise momentary behaviour: send the jog on press, send 's' on
// release. 's' is the arm's E-STOP and stops every axis, which is the correct
// conservative choice for a manual jog -- releasing the control should stop
// motion, not just the motion you were thinking about.
static char g_jog_active = 0;

static void armJog(char cmd, const char* what) {
  if (g_jog_active == cmd) return;          // already running this one
  if (g_jog_active) armSend('s', "stop previous jog");
  g_jog_active = cmd;
  armSend(cmd, what);
}

static void armJogRelease() {
  if (!g_jog_active) return;
  g_jog_active = 0;
  armSend('s', "jog released -> stop");
}

// ------------------------------------------------------------ dock tick

// One tick of the real alignment flow, at cfg::kControlPeriodMs.
//
// STOPS AT CONFIRMATION rather than letting the state machine run its own
// ClampEngage. Its clamp states wait on limit switches to learn that clamping
// finished, and those switches are on the ARM -- which reports only human text,
// nothing parseable. So the base proves alignment, hands off, and does not
// pretend to track a mechanism it cannot observe.
static void dockTick() {
  static uint32_t last_tick = 0;
  const uint32_t now = millis();
  if (now - last_tick < cfg::kControlPeriodMs) return;
  last_tick = now;

  g_edge.update(g_tof.read());
  bool present[cfg::kNumZones];
  for (size_t i = 0; i < cfg::kNumZones; ++i) present[i] = g_edge.present(i);

  g_safety.update(g_sm.alignmentConfirmed(), FaultFlags{});
  g_sm.update(present);

  // AUTO-CLAMP: no confirmation step. Alignment confirmed IS the authorisation.
  if (g_sm.alignmentConfirmed()) {
    g_docking = false;
    g_sim_aligned = true;
    pcbStopAll();
    Console.println("\n*** ALIGNED (ToF confirmed) — clamping ***");
    armSend('b', "CLAMP sequence: X cycle, Y cycle, clamp");
  }
}

// --------------------------------------------------------------- helpers

static float axisNorm(int32_t v) {
  const float f = (float)v / 512.0f;
  return (f > -0.12f && f < 0.12f) ? 0.0f : constrain(f, -1.0f, 1.0f);
}

static void announceAligned() {
  Console.printf("\n*** SIMULATED ALIGNED = %s  (NOT a real measurement) ***\n",
                 g_sim_aligned ? "TRUE" : "false");
}

// Spin each wheel in turn so the operator can see which physical wheel is which
// and whether any needs its direction flipped.
static void identify() {
  Console.println("\n# ==== WHEEL IDENTIFICATION — WHEELS OFF THE GROUND ====");
  for (uint8_t i = 0; i < 4; ++i) {
    Console.printf("#  %s  (SV=GPIO%u)\n", WHEEL[i], pins::kWheelSV[i]);
    pcbWheel(i, 0.20f);
    delay(1500);
    pcbStopAll();
    delay(700);
  }
  Console.println("# any wheel that ran BACKWARDS: select with 1-4, then 'v'");
}

static void printCal() {
  Console.printf("invert:  %s=%d  %s=%d  %s=%d  %s=%d   (selected: %s)\n",
                 WHEEL[0], (int)g_invert[0], WHEEL[1], (int)g_invert[1],
                 WHEEL[2], (int)g_invert[2], WHEEL[3], (int)g_invert[3],
                 WHEEL[g_cal_sel]);
}

static void handleConsole() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    // Passthrough wins over every base command except the toggle itself,
    // otherwise the arm's 'm', 'p', 'x' etc. would be eaten by the base.
    if (g_arm_passthrough && c != '>') {
      Serial1.write((uint8_t)c);
      continue;
    }
    if (c >= '1' && c <= '4') {
      g_cal_sel = c - '1';
      Console.printf("cal wheel -> %s\n", WHEEL[g_cal_sel]);
      continue;
    }
    switch (c) {
      // Motors are stopped before any pairing change: putting the radio into
      // discovery while a wheel is spinning is not a combination worth having.
      case 'P': pcbStopAll(); g_pairing = true;
                BP32.enableNewBluetoothConnections(true);
                Console.println("pairing ON — hold Xbox button + PAIR until the logo flashes FAST");
                break;
      case 'O': g_pairing = false;
                BP32.enableNewBluetoothConnections(false);
                Console.println("pairing OFF");
                break;
      case 'F': pcbStopAll(); BP32.forgetBluetoothKeys();
                g_pairing = true; BP32.enableNewBluetoothConnections(true);
                Console.println("forgot stored keys; pairing ON");
                break;
      case 'm': identify(); break;
      case 'p': printCal(); break;
      case 'v': g_invert[g_cal_sel] = !g_invert[g_cal_sel];
                Console.printf("%s invert -> %d\n", WHEEL[g_cal_sel],
                               (int)g_invert[g_cal_sel]);
                break;
      case 'a': g_sim_aligned = !g_sim_aligned; announceAligned(); break;
      case 'x': pcbStopAll(); armSend('s', "E-STOP arm"); g_jog_active = 0;
                Console.println("STOP (wheels + arm)"); break;
      // Full access to the arm's own command set without duplicating it here --
      // it has ~30 single-character commands and mirroring them would just
      // create a second copy to keep in sync.
      case '>': g_arm_passthrough = !g_arm_passthrough;
                Console.printf("arm passthrough %s — every key now goes %s\n",
                               g_arm_passthrough ? "ON" : "off",
                               g_arm_passthrough ? "TO THE ARM" : "to the base");
                break;
      case '?':
        Console.println("bluetooth : P pair on | O pair off | F forget keys");
        Console.println("calibrate : m identify | 1-4 select | v flip | p print");
        Console.println("            a toggle SIMULATED aligned | x STOP all");
        Console.println("arm       : > toggle passthrough (then keys go to the arm)");
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
    pcbStopAll();  // fail safe — before anything else
    Console.println("gamepad disconnected -> STOP");
  }
}

void setup() {
  pcbInit();
  pcbStopAll();

  // ToF: XSHUT sequencing on GPIO4-7, re-addressed to 0x30..0x33. Must run
  // before anything reads corners -- all four boot at 0x29 and would collide.
  g_tof_ok = g_tof.begin();
  Console.printf("ToF: %s\n", g_tof_ok ? "up (0x30-0x33)"
                                        : "FAILED — A/dock disabled, manual drive only");

  // Link to ESP-ARM. RX first, then TX, matching HardwareSerial's signature.
  Serial1.begin(115200, SERIAL_8N1, ARM_RX, ARM_TX);

  BP32.setup(&onConnect, &onDisconnect);
  BP32.enableVirtualDevice(false);   // no phantom mouse device from the pad

  Console.println("\nWheel Drive PCB — gamepad drive (pins.h / netlist map)");
  Console.printf("  SV  %u %u %u %u\n", pins::kWheelSV[0], pins::kWheelSV[1],
                 pins::kWheelSV[2], pins::kWheelSV[3]);
  Console.printf("  arm link on GPIO%u(TX)/GPIO%u(RX) -> arm UART0 @115200\n",
                 ARM_TX, ARM_RX);
  Console.println("  DRIVE  left stick move/strafe | right stick rotate");
  Console.println("         LB / RB  slower / faster (ceiling 40%)");
  Console.println("         B  STOP everything (wheels + arm E-STOP)");
  Console.println("  ARM    D-pad U/D jog X | D-pad L/R jog Y   (held = move)");
  Console.println("  DOCK   A  run real alignment -> auto-clamp on confirm");
  Console.println("         Y  override: declare aligned by eye -> clamp now");
  Console.println("         X  release / stow");
  Console.println("         LT flip X | RT flip Y");
  Console.println("  Y      toggle SIMULATED aligned");
  Console.println("  serial: ? for help");
  Console.printf("  limit %.2f  --  WHEELS OFF THE GROUND FIRST\n", LIMITS[g_limit_idx]);
  printCal();
  Console.println("  (FL/RL inverted: left motors are mounted mirrored — verified 2026-08-11)");
}

void loop() {
  handleConsole();
  BP32.update();

  // Relay anything the arm says, so its replies and errors reach the operator
  // instead of vanishing into a wire nobody is watching.
  while (Serial1.available()) Console.write((char)Serial1.read());

  float vx = 0, vy = 0, w = 0;
  bool stop = false;
  uint8_t dpad = 0;

  const bool live = g_ctl && g_ctl->isConnected() && g_ctl->isGamepad();
  if (live) {
    vx = -axisNorm(g_ctl->axisY());   // up = forward
    vy = axisNorm(g_ctl->axisX());    // right = strafe right
    w  = axisNorm(g_ctl->axisRX());   // right stick right = rotate CW
    stop  = g_ctl->b();
    dpad  = g_ctl->dpad();

    // Edge-triggered throughout: holding a button must not repeat every loop.
    static bool pL1 = false, pR1 = false, pA = false, pX = false, pY = false;
    static bool pL2 = false, pR2 = false, pB = false;
    const bool l1 = g_ctl->l1(), r1 = g_ctl->r1();
    const bool a = g_ctl->a(), x = g_ctl->x(), y = g_ctl->y(), b = g_ctl->b();
    const bool l2 = g_ctl->l2(), r2 = g_ctl->r2();

    // --- drive speed: bumpers step through LIMITS, ceiling 40% ---
    if (l1 && !pL1 && g_limit_idx > 0) {
      --g_limit_idx;
      Console.printf("limit -> %.2f\n", LIMITS[g_limit_idx]);
    }
    if (r1 && !pR1 && g_limit_idx < (sizeof(LIMITS) / sizeof(LIMITS[0])) - 1) {
      ++g_limit_idx;
      Console.printf("limit -> %.2f\n", LIMITS[g_limit_idx]);
    }

    // --- A: run the REAL alignment flow, clamping automatically on confirm ---
    if (a && !pA) {
      if (!g_tof_ok) {
        Console.println("DOCK refused — ToF did not initialise. Alignment would be blind.");
      } else {
        g_sim_aligned = false;   // a new run invalidates any previous claim
        g_docking = true;
        g_sm.handleCommand(Command::Dock);
        Console.println("DOCK started — Approach, Orient, CenterX, CenterY, Confirm.");
        Console.println("  Clamps AUTOMATICALLY on confirmation. B aborts.");
      }
    }

    if (x && !pX) armSend('c', "RELEASE: unload, flip down, stow home");
    if (l2 && !pL2) armSend('f', "flip X then retract");
    if (r2 && !pR2) armSend('F', "flip Y then retract");

    // --- stop everything: drivetrain AND arm ---
    // B already stopped the wheels; sending the arm's E-STOP on the same button
    // means one control the operator can reach without thinking about which
    // subsystem is misbehaving.
    if (b && !pB) {
      g_jog_active = 0;
      if (g_docking) { g_docking = false; g_sm.handleCommand(Command::Abort); }
      armSend('s', "E-STOP all arm motors");
    }

    // --- Y: manual override. Declare aligned by eye, clamp now. ---
    // Kept deliberately: when the ToF disagree with reality, or when exercising
    // the arm without positioning the robot, this is the way through.
    if (y && !pY) {
      if (g_docking) {
        Console.println("override refused — a real dock is running. Press B first.");
      } else {
        g_sim_aligned = true;
        announceAligned();
        armSend('b', "CLAMP sequence: X cycle, Y cycle, clamp");
      }
    }

    pL1 = l1; pR1 = r1; pA = a; pX = x; pY = y; pB = b; pL2 = l2; pR2 = r2;

    // --- D-pad jogs the arm axes (momentary, see armJog) ---
    if      (dpad & DPAD_UP)    armJog('e', "jog X extend");
    else if (dpad & DPAD_DOWN)  armJog('r', "jog X retract");
    else if (dpad & DPAD_RIGHT) armJog('E', "jog Y extend");
    else if (dpad & DPAD_LEFT)  armJog('R', "jog Y retract");
    else                        armJogRelease();
  } else {
    armJogRelease();   // pad gone: never leave a latched jog running
  }

  const float limit = LIMITS[g_limit_idx];

  // The operator always outranks the sequence. Any stick deflection, B, or a
  // lost gamepad aborts it -- an automatic motion the operator cannot interrupt
  // by doing the obvious thing is worse than no automation at all.
  if (g_docking && (!live || stop || vx != 0 || vy != 0 || w != 0)) {
    g_docking = false;
    g_sm.handleCommand(Command::Abort);
    pcbStopAll();
    Console.printf("DOCK aborted (%s)\n", !live ? "gamepad lost"
                                        : stop  ? "B pressed" : "stick input");
  }

  if (!live || stop) {
    pcbStopAll();
  } else if (g_docking) {
    dockTick();          // the state machine owns the wheels while it runs
  } else {
    pcbDriveMix(vx, vy, w, limit);
  }

  static uint32_t last = 0;
  if (millis() - last >= 500) {
    last = millis();
    Console.printf("[%s]%s lim %.2f  vx%+.2f vy%+.2f w%+.2f  duty %4u %4u %4u %4u%s\n",
                   live ? "live" : "NO PAD", g_sim_aligned ? " [SIM-ALIGNED]" : "",
                   limit, vx, vy, w,
                   g_duty[0], g_duty[1], g_duty[2], g_duty[3],
                   g_docking ? g_sm.stateName() : g_jog_active ? "  [ARM JOGGING]" : "");
  }

  delay(10);  // yield for Bluepad32 / BTstack
}
