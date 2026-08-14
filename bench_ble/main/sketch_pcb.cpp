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
// THE ARM IS REACHED BY ESP-NOW, NOT BY WIRE
// ------------------------------------------
// The three-wire UART on GPIO1/2 never passed a byte and was retired 2026-08-14.
// Commands now go out as plain NUL-terminated text over ESP-NOW on channel 1:
// "grab", "release", "home_setup", "estop", and nothing else. The arm is already
// flashed to parse exactly those, so the strings are an interface.
//
// CONTROLS
//   DRIVE   left stick   move / strafe      right stick X  rotate
//           D-pad        8-way translation  LB / RB        slower / faster
//   DOCK    A            run the real ToF alignment, auto-grab on confirm
//   ARM     X  grab      Y  release         Menu  home_setup
//   STOP    B            E-STOP: arm AND local wheels
//   SENSE   L3 ToF stream    R3 read corners    View capture ToF offsets
//
// SAFETY
//   * no gamepad at boot -> motors stay disabled; nothing spins on power-up
//   * gamepad disconnect -> wheels stop
//   * B stops BOTH boards, and bypasses the busy interlock: an E-stop that a
//     status message can suppress is not an E-stop
//   * drive speed is capped at 40% -- there is no stall detection on this base
//   WHEELS OFF THE GROUND.
#include <Arduino.h>
#include <Bluepad32.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

#include "bench_mix.h"   // pure mecanum mix, shared with the bench rigs
#include "config.h"
#include "pins.h"        // NETLIST-DERIVED. never hand-type a pin table.

// The REAL alignment stack, reused rather than reimplemented. These are the
// same classes main.cpp runs and the same ones covered by the 70 host tests --
// a second copy of a docking state machine is the last thing this project needs.
#include "ArduinoClock.h"
#include "CornerEdgeDetector.h"
#include "DockFrame.h"
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
// Set ONLY by a real ToF confirmation now. It was a manual override on Y until
// 2026-08-14, when Y became the arm's "release"; the status line still shows it
// because "this base believes it is aligned" is worth seeing at a glance.
static bool g_sim_aligned = false;

// SPEED LIMITS -- CEILING IS 40%, ON PURPOSE.
// There is no encoder feedback and no stall detection on this base (no ALARM
// terminal, encoders deferred), so nothing in software can tell a jammed wheel
// from a loaded one. 40% of a drivetrain that reaches ~1400 mm/s is plenty for
// manual work and leaves the operator time to react.
static const float LIMITS[] = {0.10f, 0.20f, 0.30f, 0.40f};
static uint8_t g_limit_idx = 0;

// ---- link to ESP-ARM: ESP-NOW, replacing the three-wire UART ---------------
//
// The UART link on GPIO1/2 never passed a byte and was retired 2026-08-14. This
// is radio instead: no wires, no crossing to get backwards, and it works whether
// or not the arm's USB cable is plugged in.
//
// THE COST IS REAL AND WORTH STATING. Bluepad32/BTstack owns the 2.4 GHz radio
// for the gamepad; bringing up WiFi STA makes them share it through the
// coexistence scheduler. Expect the pad to be slightly less responsive and to
// drop marginally more often than it did on the UART build. If that becomes
// intolerable the answer is to fix the wires, not to tune this.
//
// PROTOCOL: plain text, NUL-terminated, four commands and no more. No JSON, no
// structs, no versioning -- the arm's receiver is already flashed and parses
// exactly these, so the strings are an interface, not an implementation detail.
//
//     "grab"        "release"        "home_setup"        "estop"
//
// Replies come back as text too: done_grab, done_release, done_home, busy,
// stopped, failed:<reason>, rejected:<reason>.
static const uint8_t ARM_ESP_MAC[6] = {0x3C, 0xDC, 0x75, 0x5C, 0x8B, 0x08};

// This board's own MAC, checked at boot. Flashing base firmware onto the arm (or
// vice versa) is a five-second mistake that presents as "the link is dead", and
// we have already spent an evening on a link that was dead for other reasons.
static const uint8_t SELF_ESP_MAC[6] = {0x14, 0xC1, 0x9F, 0x3B, 0x7B, 0xE4};

// WiFi channel. ESP-NOW peers must agree, and since neither board joins an AP
// nothing will move us off it -- but it has to be set explicitly, because the
// default depends on prior NVS state rather than on anything in this file.
static constexpr uint8_t ARM_WIFI_CHANNEL = 1;

static bool g_espnow_up = false;
static uint32_t g_arm_replies = 0;

// Last text the arm sent. Read by the busy interlock below.
static char g_arm_status[32] = "";

// BUSY INTERLOCK, modelled on what the arm ACTUALLY does (Arm_code, reviewed
// 2026-08-14 and pinned as the `arm` submodule).
//
// The arm accepts exactly ONE command at a time. Its buffer is a single slot the
// receive callback overwrites unconditionally, so a second command sent before
// it has serviced the first is silently lost -- no reply, no error. And a grab
// or release runs X then Y sequentially with 8 s timeouts per phase, so it can
// legitimately be busy for ~40 SECONDS.
//
// So busy is latched WHEN WE SEND, not when the arm complains. Waiting for the
// arm to answer "busy" would mean the second press has already been fired into
// a slot that may be overwritten, and it made every other press get through --
// harmless, since the arm rejects them, but it lied to the operator about what
// had been sent.
//
// Cleared by any terminal reply: done_*, failed:*, rejected:*, stopped.
static bool g_arm_busy = false;
static uint32_t g_arm_busy_since = 0;

// ...and by a timeout, because the arm can go quiet without telling us. Its
// serial console has its own `stop`, which calls emergencyStopEverything(false)
// -- sendStatus FALSE. Anyone at the arm's USB port can abort a workflow and our
// interlock would never hear about it, latching the controls forever.
//
// 45 s covers the ~40 s worst case with margin.
static constexpr uint32_t ARM_BUSY_TIMEOUT_MS = 45000;

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

// ACTIVE BRAKING ON RELEASE.
//
// This file used to coast on zero command -- drop EN, leave BRK released, let
// the motor free-wheel. On a base this heavy that is a long, vague stop, and it
// makes every centring test ambiguous: you cannot tell a controller that
// overshot from one that stopped correctly and then rolled.
//
// So a zero command now asserts BRK for a fixed window. Two details matter:
//
// EN STAYS ASSERTED WHILE BRAKING. The BLD-120A's brake acts through the output
// stage, so a disabled driver cannot brake -- releasing EN and asserting BRK in
// the same breath would just coast with extra steps.
//
// THE WINDOW IS TIMED, NOT LATCHED. Holding brake plus enable indefinitely
// leaves the output stage dissipating into a stalled motor for as long as the
// robot sits idle. 400 ms stops it, then both lines release to a genuine idle.
static constexpr uint32_t BRAKE_MS = 400;
static uint32_t g_brake_until[4] = {0, 0, 0, 0};
static bool g_moving[4] = {false, false, false, false};

// Edge-triggered: the brake window opens on the MOVING -> STOPPED transition
// only. pcbStopAll() runs every loop while no pad is connected, and a
// level-triggered version would re-arm the window forever and hold EN+BRK
// asserted for as long as the robot is parked.
static void pcbIdle(uint8_t i) {
  ledcWrite(pins::kWheelSV[i], 0);
  g_duty[i] = 0;
  if (g_moving[i]) {
    g_moving[i] = false;
    g_brake_until[i] = millis() + BRAKE_MS;
  }
  const bool braking = (int32_t)(g_brake_until[i] - millis()) > 0;
  digitalWrite(pins::kWheelEN[i], braking ? ASSERT_LVL : RELEASE_LVL);
  digitalWrite(pins::kWheelBRK[i], braking ? ASSERT_LVL : RELEASE_LVL);
}

static void pcbStopAll() {
  for (uint8_t i = 0; i < 4; ++i) pcbIdle(i);
}

// Signed command in [-1, 1]. Zero hands off to pcbIdle above, which brakes for
// BRAKE_MS and then releases.
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
  if (duty == 0) {
    pcbIdle(i);   // brake for BRAKE_MS, then release
    return;
  }

  g_moving[i] = true;
  g_brake_until[i] = 0;   // a fresh command cancels any brake still running
  digitalWrite(pins::kWheelEN[i], ASSERT_LVL);
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
    // ROTATED. The state machine advances along +vx in ITS frame, which is the
    // robot's strafe axis -- the sequence crabs under the trolley while manual
    // driving stays nose-first. toDockFrame() in dockTick() permutes the corners
    // to match; the two must always change together.
    const DriveCommand r = toRobotFrame(c, cfg::kDockStrafeRight);
    // limit 1.0: the state machine's own config already sets its speeds, and
    // scaling them again by the operator's drive limit would silently change
    // the docking behaviour whenever someone touched a bumper.
    pcbDriveMix(r.vx, r.vy, r.omega, 1.0f);
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

// D-PAD MODE. Defaults to DRIVE because that is what the base needs during
// bring-up: eight exact directions, each a pure axis command, so an asymmetry
// can be attributed to one corner instead of guessed at from a stick angle.
// Serial 'j' hands the D-pad back to the arm jog.
// Kept as a named constant rather than deleted: the D-pad block below reads far
// better with the intent spelled out, and the ARM JOG alternative it used to
// select is gone with the UART (the arm's per-axis jog characters are not part
// of the four-command ESP-NOW protocol).
static constexpr bool g_dpad_drive = true;

// ------------------------------------------------------------ ToF debug
//
// Centring is BLIND WITHOUT THIS. DockingStateMachine's only sensor input is
// four booleans; if one corner is mis-mapped or its band is wrong, every state
// still runs and the robot simply stops in the wrong place, with nothing on the
// console to say why. These commands make the four booleans visible.

// PCB silkscreen labels for the ToF connectors, so the console names what is
// printed beside the header rather than a GPIO the operator has to look up.
// Confirmed on the assembled base 2026-08-13: labels 4/6/7/8 = FL/FR/RL/RR.
// The labels are NOT the GPIOs -- label 8 sits on GPIO7, and GPIO8 is the I2C
// SDA line, which is why the numbers cannot be read as a pin map.
static const char* TOF_LABEL[4] = {"4", "6", "7", "8"};
static const char* CORNER[4] = {"FL", "FR", "RL", "RR"};

static AlignmentFrame g_frame;   // last raw read, kept for the valid flags
static bool g_tof_stream = false;

// THE ONLY PLACE THE ToF BUS IS READ. Both the docking tick and the operator's
// stream go through here, so the console shows exactly the frame the state
// machine acted on. A second independent read would sample a different instant
// and make any disagreement between them impossible to diagnose.
static uint32_t g_tof_frame_us = 0;   // cost of the last read(), measured

static void tofPoll() {
  const uint32_t t0 = micros();
  g_frame = g_tof.read();
  g_tof_frame_us = micros() - t0;
  g_edge.update(g_frame);
}

// Raw mm alongside the debounced boolean, because they fail differently:
//   TIMEOUT   the sensor did not answer      -> wiring / address / power
//   8190 mm   answered, saw nothing in range -> aimed at open air
//   in band   answered, board overhead       -> PRESENT after `debounce` samples
// Reading only "present" cannot tell the first two apart, and they need
// completely different fixes.
static void tofPrint() {
  const tb::CornerConfig cc = cfg::makeCornerConfig();
  // Frame time is the number that governs centring accuracy, so it is printed
  // on every line rather than buried in a one-off benchmark. Edge detection
  // lags by frame x CornerConfig::debounce, and CenterX turns that lag into a
  // systematic forward bias -- both its edges are found while driving +x, so
  // the bisection cannot cancel it the way CenterY's opposed sweeps do.
  const float lag_mm = (g_tof_frame_us / 1000.0f) * cc.debounce *
                       (cfg::makeDockConfig().centre_speed *
                        cfg::makeOdometryCal().max_lin_mm_s) / 1000.0f;
  Console.printf("ToF %lu us/frame (edge lag ~%.1f mm at centre speed)  band %u-%u mm |",
                 (unsigned long)g_tof_frame_us, lag_mm, cc.band_min_mm, cc.band_max_mm);
  for (size_t i = 0; i < cfg::kNumZones; ++i) {
    // THREE OUTCOMES, NOT TWO. AlignmentFrame::valid folds together "the sensor
    // did not answer" and "the sensor answered, nothing in range", and those
    // demand opposite responses -- one is a fault, the other is the normal
    // reading for a sensor pointed at open air. Printing the raw mm alongside is
    // what separates them: the VL53L0X returns 65535 on a real timeout and 8190
    // when it ranged successfully and found no target.
    const uint16_t mm = g_edge.mm(i);
    Console.printf("  %s(lbl%s/IO%u) ", CORNER[i], TOF_LABEL[i], pins::kTofXSHUT[i]);
    if (mm == 65535)     Console.print("TIMEOUT      ");   // no answer -- a fault
    else if (mm >= 8000) Console.print("  no target   ");  // 8190: ranged, saw nothing
    else                 Console.printf("%4u mm %-7s", mm,
                                        g_edge.present(i) ? "PRESENT" : "-");
  }
  Console.println();
}

// CAPTURE PER-CORNER OFFSETS, referenced to FL.
//
// Hold the robot at a fixed standoff under a flat surface and run this. It
// averages N frames of RAW readings and solves offset[i] = mean[FL] - mean[i],
// so every corner afterwards reports what FL reports for the same gap.
//
// Referenced to a CORNER, not to a measured true distance, deliberately: the
// operator can read four agreeing numbers off the console and see immediately
// whether a corner is drifting, without having to trust a caliper reading taken
// at an awkward angle under a trolley. If absolute truth is wanted later, shift
// all four by the same constant -- the relative solution does not change.
//
// Blocks for N frames (~1.5 s at the measured 93 ms/frame), so wheels are
// stopped first and it refuses to run during a dock.
static void tofCalibrate() {
  static constexpr int N = 16;
  pcbStopAll();

  // Zero first: the capture must see RAW readings, or a second run would
  // compound offsets onto the ones already applied.
  for (size_t i = 0; i < cfg::kNumZones; ++i) g_tof.setOffset(i, 0);

  Console.printf("\n# ToF offset capture — %d frames, hold still...\n", N);
  uint32_t sum[cfg::kNumZones] = {0};
  uint16_t n[cfg::kNumZones] = {0};
  for (int k = 0; k < N; ++k) {
    const AlignmentFrame f = g_tof.read();
    for (size_t i = 0; i < cfg::kNumZones; ++i)
      if (f.zones[i].valid) { sum[i] += f.zones[i].mm; ++n[i]; }
  }

  // A corner with no valid samples cannot be solved for, and guessing an offset
  // for it would be worse than leaving it uncorrected.
  for (size_t i = 0; i < cfg::kNumZones; ++i) {
    if (n[i] == 0) {
      Console.printf("# ABORTED — %s saw no target in %d frames. Offsets unchanged.\n",
                     CORNER[i], N);
      for (size_t j = 0; j < cfg::kNumZones; ++j) g_tof.setOffset(j, cfg::kTofOffsetMm[j]);
      return;
    }
  }

  float mean[cfg::kNumZones];
  for (size_t i = 0; i < cfg::kNumZones; ++i) mean[i] = (float)sum[i] / n[i];

  Console.print("# raw means:");
  for (size_t i = 0; i < cfg::kNumZones; ++i)
    Console.printf("  %s %.1f mm", CORNER[i], mean[i]);
  Console.println();

  int16_t off[cfg::kNumZones];
  for (size_t i = 0; i < cfg::kNumZones; ++i) {
    off[i] = (int16_t)lroundf(mean[0] - mean[i]);   // FL is the reference
    g_tof.setOffset(i, off[i]);
  }

  Console.println("\n// paste into include/config.h, replacing kTofOffsetMm");
  Console.printf("constexpr int16_t kTofOffsetMm[4] = {%d, %d, %d, %d};  // FL FR RL RR\n",
                 off[0], off[1], off[2], off[3]);
  Console.println("# applied now — press R3 to confirm all four agree.");
  Console.println("# NOT persistent: paste the line above or it is lost at reset.\n");
}

// Fresh frame, then how many corners read present. Used to refuse a dock that
// would start from under the trolley -- polls rather than trusting g_edge,
// which may be seconds stale if nothing has been reading the bus.
//
// Debounce means one poll cannot flip a corner on its own, so this deliberately
// reports the DEBOUNCED state: a corner that a single frame would have lit is
// not one the state machine would have acted on either.
static size_t tofPollAndCountPresent() {
  tofPoll();
  size_t n = 0;
  for (size_t i = 0; i < cfg::kNumZones; ++i) n += g_edge.present(i) ? 1 : 0;
  return n;
}

// ------------------------------------------------------------- arm link

// Delivery result. This is LINK-LAYER only: SUCCESS means the arm's radio
// acknowledged the frame, not that the arm liked the command or acted on it.
// The real answer arrives asynchronously in armOnRecv below.
static void armOnSent(const uint8_t* mac, esp_now_send_status_t status) {
  (void)mac;
  if (status != ESP_NOW_SEND_SUCCESS)
    Console.println("  ESP-NOW: NOT DELIVERED — arm out of range, off, or wrong channel");
}

// Signature is the ESP-IDF 5.x form (esp_now_recv_info_t). This tree builds
// against arduino-esp32 3.2.1 / IDF 5.4; the older 4.x callback took a bare MAC
// pointer, so this will not compile unchanged on an older framework.
static void armOnRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
  (void)info;
  if (len <= 0) return;

  // Copy defensively rather than trusting the sender's NUL: len is what the
  // radio actually delivered, and a missing terminator would otherwise walk off
  // the end of the driver's buffer.
  size_t n = (size_t)len;
  if (n >= sizeof(g_arm_status)) n = sizeof(g_arm_status) - 1;
  memcpy(g_arm_status, data, n);
  g_arm_status[n] = '\0';
  ++g_arm_replies;

  Console.printf("arm -> \"%s\"\n", g_arm_status);

  // Anything except "busy" is terminal -- done_*, stopped, failed:, rejected:.
  // The arm is free again whether it succeeded or not, so the controls come back
  // either way. "busy" means our command was discarded and the PREVIOUS workflow
  // is still running, so the latch stays.
  if (strcmp(g_arm_status, "busy") != 0) g_arm_busy = false;
}

// Bring up WiFi far enough for ESP-NOW, and no further.
//
// We never join an AP: ESP-NOW is a link-layer protocol, so STA mode plus a
// fixed channel is the whole requirement. Not calling WiFi.begin() also means
// nothing will ever scan or roam us off channel 1 behind our back.
static bool armLinkBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false, false);   // ensure no stored AP creds pull us onto another channel

  uint8_t mac[6] = {0};
  esp_wifi_get_mac(WIFI_IF_STA, mac);
  Console.printf("ESP-NOW: this board is %02X:%02X:%02X:%02X:%02X:%02X\n",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  if (memcmp(mac, SELF_ESP_MAC, 6) != 0) {
    Console.printf("  *** EXPECTED %02X:%02X:%02X:%02X:%02X:%02X — WRONG BOARD? ***\n",
                   SELF_ESP_MAC[0], SELF_ESP_MAC[1], SELF_ESP_MAC[2],
                   SELF_ESP_MAC[3], SELF_ESP_MAC[4], SELF_ESP_MAC[5]);
    Console.println("  (harmless if the board was replaced — update SELF_ESP_MAC)");
  }

  // Channel must be set AFTER the interface is started, or it does not stick.
  esp_wifi_set_channel(ARM_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Console.println("ESP-NOW: init FAILED — arm commands will not be sent");
    return false;
  }
  esp_now_register_send_cb(armOnSent);
  esp_now_register_recv_cb(armOnRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ARM_ESP_MAC, 6);
  peer.channel = ARM_WIFI_CHANNEL;   // 0 would mean "whatever the interface is on"
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Console.println("ESP-NOW: add_peer FAILED — check ARM_ESP_MAC");
    return false;
  }

  Console.printf("ESP-NOW: up on channel %u, arm peer %02X:%02X:%02X:%02X:%02X:%02X\n",
                 ARM_WIFI_CHANNEL, ARM_ESP_MAC[0], ARM_ESP_MAC[1], ARM_ESP_MAC[2],
                 ARM_ESP_MAC[3], ARM_ESP_MAC[4], ARM_ESP_MAC[5]);
  return true;
}

// SENDS THE TRAILING NUL. The arm parses C strings, so the terminator is part of
// the message, not an artefact of how we happen to store it.
static void armSend(const char* cmd, const char* what) {
  if (!g_espnow_up) {
    Console.printf("arm <- \"%s\" NOT SENT (ESP-NOW down) (%s)\n", cmd, what);
    return;
  }
  const esp_err_t e = esp_now_send(ARM_ESP_MAC,
                                   (const uint8_t*)cmd, strlen(cmd) + 1);
  Console.printf("arm <- \"%s\"  (%s)%s\n", cmd, what,
                 e == ESP_OK ? "" : "  [QUEUE FAILED]");
}

// Ordinary commands go through the busy interlock; E-stop does not.
static void armCommand(const char* cmd, const char* what) {
  if (g_arm_busy) {
    Console.printf("arm <- \"%s\" HELD — arm busy %lus (%s)\n",
                   cmd, (unsigned long)((millis() - g_arm_busy_since) / 1000), what);
    Console.println("  it takes ~40 s for grab/release. B always gets through.");
    return;
  }
  g_arm_busy = true;
  g_arm_busy_since = millis();
  armSend(cmd, what);
}

// Release the latch if the arm never answers. See ARM_BUSY_TIMEOUT_MS.
static void armBusyTick() {
  if (!g_arm_busy) return;
  if (millis() - g_arm_busy_since < ARM_BUSY_TIMEOUT_MS) return;
  g_arm_busy = false;
  Console.println("\narm: no reply in 45 s — releasing the interlock.");
  Console.println("  the arm may have been stopped from its own serial console,");
  Console.println("  which reports nothing back. Its state is now UNKNOWN.");
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

  tofPoll();
  bool robot_corners[cfg::kNumZones];
  for (size_t i = 0; i < cfg::kNumZones; ++i) robot_corners[i] = g_edge.present(i);

  // Into the rotated frame: the machine's "leading pair" is the robot's
  // right-hand side, because the sequence crabs in. Must use the same
  // kDockStrafeRight as GamepadDrive::move, or it drives one way and reads
  // edges from the other -- which bisects nonsense rather than failing loudly.
  bool present[cfg::kNumZones];
  toDockFrame(robot_corners, present, cfg::kDockStrafeRight);

  g_safety.update(g_sm.alignmentConfirmed(), FaultFlags{});
  const DockState before = g_sm.state();
  g_sm.update(present);

  // LOG EVERY TRANSITION, with the corner pattern and pose that caused it.
  //
  // The sequence is a chain of edge triggers -- Approach ends when a front
  // corner lights, CenterX bisects between the two x positions where the front
  // pair flipped. Printed after the fact, a bad run is just "it stopped in the
  // wrong place"; printed at each transition, the exact corner and the exact
  // odometry reading that drove the decision are both on screen.
  if (g_sm.state() != before) {
    const Pose2D p = g_odom.pose();
    // ROBOT-frame corners, not the rotated ones the machine sees. The operator
    // is looking at a physical robot; printing dock-frame slots would mean
    // mentally un-rotating every line to work out which sensor is dark.
    Console.printf("DOCK  %-11s  FL%c FR%c RL%c RR%c  x%+7.1f y%+7.1f th%+5.2f\n",
                   g_sm.stateName(),
                   robot_corners[0] ? '*' : '.', robot_corners[1] ? '*' : '.',
                   robot_corners[2] ? '*' : '.', robot_corners[3] ? '*' : '.',
                   p.x_mm, p.y_mm, p.theta_rad);

    // THE TROLLEY CALIBRATES THE SPEED CONSTANT.
    //
    // x_near is stamped on entry to CenterX and x_target on entry to CenterY,
    // and x_target = 0.5*(x_near + x_far) - front_offset, so
    //
    //     span = 2 * (x_target - x_near + front_offset)
    //
    // recovers the traverse without the state machine having to expose x_far.
    // That span IS the trolley depth, which is known independently -- so every
    // successful CenterX measures max_lin_mm_s through the same sensors, at the
    // same speed, over the same motion the docking actually performs. Far better
    // than a stopwatch over a run that included acceleration from rest.
    // Keyed on LEAVING CenterX: with DockingConfig::centre_lateral false the
    // sequence goes straight to Confirm and CenterY never happens.
    //
    // The marks come straight off the state machine now rather than being
    // reconstructed by algebra. That distinction matters when the complaint is
    // "it overshot": near/far/target say whether the EDGES were found in the
    // wrong place or the drive simply failed to reach a correct target, and no
    // amount of staring at a final pose separates those two.
    if (before == DockState::CenterX) {
      const float span = g_sm.markFar() - g_sm.markNear();
      // lo = the TRAILING pair covering the near edge, which is what the target
      // is now bisected from (see DockingConfig::centre_opposed_pairs). near/far
      // are kept because their span is what calibrates max_lin_mm_s.
      Console.printf("      near %+.1f  lo %+.1f  far %+.1f  span %.1f  target %+.1f  ended %+.1f\n",
                     g_sm.markNear(), g_sm.markLo(), g_sm.markFar(), span,
                     g_sm.markTarget(), p.x_mm);
      if (span > 1.0f) {
        const float cal = cfg::makeOdometryCal().max_lin_mm_s;
        Console.printf("      expected span %.0f mm (860 trolley + 2x28 cone) -> max_lin_mm_s "
                       "config has %.0f\n",
                       cfg::kExpectedSpanMm, cal * cfg::kExpectedSpanMm / span, cal);
        if (span < 0.6f * cfg::kTrolleyDepthMm)
          Console.println("      *** SPAN FAR TOO SHORT — the leading pair went dark "
                          "mid-traverse. False edge, not a real one.");
      }
    }
    if (g_sm.state() == DockState::Fault) {
      Console.printf("DOCK  FAULT: %s\n", g_sm.lastReason());
      Console.println("      (press B to abort and clear, then 't' to see the corners)");
      g_docking = false;
      pcbStopAll();
    }
  }

  // AUTO-CLAMP: no confirmation step. Alignment confirmed IS the authorisation.
  if (g_sm.alignmentConfirmed()) {
    g_docking = false;
    g_sim_aligned = true;
    pcbStopAll();
    Console.println("\n*** ALIGNED (ToF confirmed) — clamping ***");
    armCommand("grab", "auto-clamp on ToF confirmation");
    // RETURN THE MACHINE TO IDLE, or the next dock cannot start.
    //
    // Confirmation moves it into ClampEngage, and we then stop ticking it -- so
    // it sits there. Command::Dock is only honoured from Idle, so pressing A
    // again did not restart anything; it just resumed ticking a ClampEngage
    // whose deadline had long since passed, and the first tick reported
    // "clamp timeout" (observed 2026-08-13). The clamp lives on the arm and is
    // driven by the 'b' above, so this machine has nothing left to do here.
    g_sm.handleCommand(Command::Abort);
  }
}

// --------------------------------------------------------------- helpers

static float axisNorm(int32_t v) {
  const float f = (float)v / 512.0f;
  return (f > -0.12f && f < 0.12f) ? 0.0f : constrain(f, -1.0f, 1.0f);
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

// *** THIS FUNCTION NEVER RUNS. KEEP THAT IN MIND BEFORE ADDING TO IT. ***
//
// bench_ble builds Arduino as an ESP-IDF component with Bluepad32 owning the
// console -- sdkconfig.defaults sets CONFIG_BLUEPAD32_USB_CONSOLE_ENABLE=y and
// says in its own comment "Arduino Serial conflicts with the console". So:
//
//   * `Console` is OUTPUT ONLY. ArduinoConsole.h declares print/println/printf/
//     write and nothing else -- there is no available() and no read().
//   * `Serial` is never begin()-ed, and must not be: it would fight the IDF
//     console for the same peripheral. Serial.available() is therefore always
//     false and this whole loop is unreachable.
//   * stdin belongs to Bluepad32's own REPL (uni_console_init), which is what
//     actually receives anything typed into the monitor.
//
// Diagnostics that need an operator therefore go on GAMEPAD buttons, which is
// the only input path this firmware really has. See the thumbL/thumbR handling
// in loop(). The body below is left in place because it is the natural home for
// commands if the console is ever reconfigured -- not because it works.
static void handleConsole() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
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
      case 't': tofPoll(); tofPrint(); break;
      case 'T': g_tof_stream = !g_tof_stream;
                Console.printf("ToF stream %s\n", g_tof_stream ? "ON (5 Hz)" : "off");
                break;
      case 'x': pcbStopAll(); armSend("estop", "E-STOP arm + wheels");
                Console.println("STOP (wheels + arm)"); break;
      case '?':
        Console.println("bluetooth : P pair on | O pair off | F forget keys");
        Console.println("calibrate : m identify | 1-4 select | v flip | p print");
        Console.println("            x STOP all");
        Console.println("sensors   : t read the 4 corners once | T stream at 5 Hz");
        break;
      default: break;
    }
  }
}

static void onConnect(ControllerPtr c) {
  if (!g_ctl) {
    g_ctl = c;
    Console.printf("gamepad connected: %s\n", c->getModelName());
    // STOP SCANNING once we have the pad we need.
    //
    // Bluepad32 scans continuously by default, and anything BLE in the room it
    // cannot pair with -- a Logitech mouse was the culprit here -- gets retried
    // forever: discover, connect, fail identity resolution, disconnect, repeat
    // several times a second. That floods the console badly enough to hide the
    // ToF stream, and spends radio time for nothing.
    //
    // Already-bonded devices still reconnect with scanning off, so the pad is
    // unaffected; onDisconnect re-opens it so a new pad can still be paired
    // without a reflash (there is no serial console to type 'P' into).
    BP32.enableNewBluetoothConnections(false);
    g_pairing = false;
  }
}

static void onDisconnect(ControllerPtr c) {
  if (g_ctl == c) {
    g_ctl = nullptr;
    pcbStopAll();  // fail safe — before anything else
    BP32.enableNewBluetoothConnections(true);   // re-open so a pad can be paired
    g_pairing = true;
    Console.println("gamepad disconnected -> STOP  (scanning re-enabled)");
  }
}

void setup() {
  pcbInit();
  pcbStopAll();

  // *** START THE I2C BUS BEFORE ANY SENSOR. ***
  // Vl53l0xArray does not do this itself -- the bus is shared with the IMU, the
  // current-sense ADC and the encoder mux, so exactly one place owns it and the
  // rest join. Omitting it does not error: every sensor's init() simply fails
  // against an unconfigured peripheral, which looks identical to four dead
  // connectors. (It was missing here until 2026-08-13; main.cpp always had it.)
  Wire.begin(pins::kI2C_SDA, pins::kI2C_SCL);

  // Bus inventory first, so a ToF failure can be told apart from a dead bus
  // before any conclusion is drawn about the sensors themselves.
  Console.printf("I2C on SDA=%u SCL=%u — responders:", pins::kI2C_SDA, pins::kI2C_SCL);
  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) { Console.printf(" 0x%02X", addr); ++found; }
  }
  Console.println(found ? "" : "  (NONE — bus is dead: check pull-ups, 3V3, GND)");

  // ToF: XSHUT sequencing on GPIO4-7, re-addressed to 0x30..0x33. Must run
  // before anything reads corners -- all four boot at 0x29 and would collide.
  g_tof_ok = g_tof.begin();
  // Offsets AFTER begin(): begin() re-inits each sensor and the offsets live in
  // the array, not the sensor, so ordering only matters for clarity here.
  for (size_t i = 0; i < cfg::kNumZones; ++i) g_tof.setOffset(i, cfg::kTofOffsetMm[i]);
  Console.printf("ToF: %u/%u up —", (unsigned)g_tof.okCount(), (unsigned)cfg::kNumZones);
  for (size_t i = 0; i < cfg::kNumZones; ++i)
    Console.printf("  %s(lbl%s/IO%u) %s", CORNER[i], TOF_LABEL[i], pins::kTofXSHUT[i],
                   g_tof.ok(i) ? "ok" : "FAILED");
  Console.println();
  if (!g_tof_ok)
    Console.println("  docking disabled (needs all four); streaming still works on whatever is up");

  // Link to ESP-ARM over ESP-NOW. Brought up AFTER the ToF so a WiFi failure
  // cannot take the sensors down with it -- driving and docking must survive a
  // dead arm link, which is exactly the case this replaced.
  g_espnow_up = armLinkBegin();

  BP32.setup(&onConnect, &onDisconnect);
  BP32.enableVirtualDevice(false);   // no phantom mouse device from the pad

  Console.println("\nWheel Drive PCB — gamepad drive (pins.h / netlist map)");
  Console.printf("  SV  %u %u %u %u\n", pins::kWheelSV[0], pins::kWheelSV[1],
                 pins::kWheelSV[2], pins::kWheelSV[3]);
  Console.println("  DRIVE  left stick move/strafe | right stick rotate");
  Console.println("         LB / RB  slower / faster (ceiling 40%)");
  Console.println("  D-PAD  8-way DRIVE: U/D forward/back, L/R strafe, corners diagonal");
  Console.println("  DOCK   A  run real alignment -> auto-clamp (\"grab\") on confirm");
  Console.println("  ARM    X  grab      Y  release      Menu  home_setup");
  Console.println("         B  E-STOP — arm AND local wheels, always sends");
  Console.println("  SENSE  L3 (left stick click)  toggle ToF stream");
  Console.println("         R3 (right stick click) read the 4 corners once");
  Console.println("         View  capture per-corner ToF offsets");
  Console.println("  NOTE   the serial console is OUTPUT ONLY — Bluepad32 owns");
  Console.println("         stdin, so typed keys do nothing here. Use the pad.");
  Console.printf("  limit %.2f  --  WHEELS OFF THE GROUND FIRST\n", LIMITS[g_limit_idx]);
  printCal();
  Console.println("  (FL/RL inverted: left motors are mounted mirrored — verified 2026-08-11)");
}

void loop() {
  handleConsole();
  BP32.update();

  // Arm replies arrive on an ESP-NOW callback, not by polling. The only thing
  // to service is the interlock's own timeout.
  armBusyTick();

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
    static bool pL2 = false, pR2 = false, pB = false, pTL = false, pTR = false;
    const bool l1 = g_ctl->l1(), r1 = g_ctl->r1();
    const bool a = g_ctl->a(), x = g_ctl->x(), y = g_ctl->y(), b = g_ctl->b();
    const bool l2 = g_ctl->l2(), r2 = g_ctl->r2();
    const bool tl = g_ctl->thumbL(), tr = g_ctl->thumbR();
    const bool sel = g_ctl->miscSelect();   // View button — ToF offset capture
    const bool menu = g_ctl->miscStart();   // Menu button — arm home/setup

    static bool pMenu = false;
    if (menu && !pMenu) armCommand("home_setup", "arm: home and set up");
    pMenu = menu;

    // View: capture per-corner offsets against whatever is overhead right now.
    // Refused mid-dock: it blocks ~1.5 s and zeroes the offsets while running,
    // which would corrupt the frames the state machine is acting on.
    static bool pSel = false;
    if (sel && !pSel) {
      if (g_docking) Console.println("offset capture refused — dock running. Press B first.");
      else if (g_tof.okCount() < cfg::kNumZones)
        Console.println("offset capture refused — needs all four corners up.");
      else tofCalibrate();
    }
    pSel = sel;

    // --- ToF diagnostics on the STICK CLICKS ---
    //
    // On the pad because the serial console cannot receive input at all (see
    // handleConsole). L3/R3 are the only buttons still free, and they are
    // awkward enough to press that neither will be hit by accident while
    // driving -- which matters, since a poll blocks the loop for ~90 ms.
    //
    // Gated on ANY sensor being up, not all four. Diagnostics have to work on
    // broken hardware -- that is the only time they matter. Docking still needs
    // all four; that gate is on the A button.
    if (tl && !pTL) {
      if (g_tof.okCount() == 0) {
        Console.println("no ToF came up — nothing to stream. Check the I2C line at boot.");
      } else {
        g_tof_stream = !g_tof_stream;
        Console.printf("ToF stream %s (%u sensor(s) up)\n",
                       g_tof_stream ? "ON (5 Hz)" : "off", (unsigned)g_tof.okCount());
      }
    }
    if (tr && !pTR) {
      if (g_tof.okCount() == 0)
        Console.println("no ToF came up — check XSHUT GPIO4-7, 3V3/GND, and the boot I2C scan.");
      else { if (!g_docking) tofPoll(); tofPrint(); }
    }

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
      } else if (tofPollAndCountPresent() > 0) {
        // MUST START WITH EVERY CORNER CLEAR.
        //
        // Approach exits the instant a leading corner lights, and Orient then
        // wants exactly one of the pair. Starting with a corner already present
        // fires that on tick one at a meaningless position, and Orient hunts on
        // a corner that was never an edge crossing -- which reads as "it pivots
        // endlessly the moment I press A".
        //
        // The crossing IS the measurement, so it has to happen under power.
        Console.print("DOCK refused — start OUTSIDE the trolley. Already present:");
        for (size_t i = 0; i < cfg::kNumZones; ++i)
          if (g_edge.present(i)) Console.printf("  %s (%u mm)", CORNER[i], g_edge.mm(i));
        Console.println("\n  back out until R3 shows all four clear, then press A.");
      } else {
        g_sim_aligned = false;   // a new run invalidates any previous claim
        g_docking = true;
        g_sm.handleCommand(Command::Dock);
        Console.println("DOCK started — Approach, Orient, CenterX, CenterY, Confirm.");
        Console.println("  Clamps AUTOMATICALLY on confirmation. B aborts.");
      }
    }

    // X = grab, Y = release. LT/RT no longer send anything: the ESP-NOW
    // protocol is four commands, and the old per-axis flip/jog characters do not
    // exist on the arm any more.
    if (x && !pX) armCommand("grab", "arm: clamp the trolley");

    // --- stop everything: drivetrain AND arm ---
    // B already stopped the wheels; sending the arm's E-STOP on the same button
    // means one control the operator can reach without thinking about which
    // subsystem is misbehaving.
    if (b && !pB) {
      if (g_docking) { g_docking = false; g_sm.handleCommand(Command::Abort); }
      // BYPASSES armCommand's busy interlock on purpose. An E-stop that can be
      // suppressed by a status message is not an E-stop. The wheels are already
      // stopped further down by the `stop` branch; this is the other board.
      //
      // *** THE STRING MUST BE EXACTLY "estop" -- lowercase, no whitespace. ***
      // The arm has two paths for it. processWirelessCommand() trims and
      // lowercases, so "ESTOP\n" works there -- but that only runs from its
      // loop(), and its servo ramps BLOCK for ~1.8 s at a time. The mid-motion
      // abort uses a raw strncmp against the untrimmed buffer, so any variant
      // silently downgrades E-stop from "stops the servo now" to "stops it when
      // the ramp finishes". (Arm_code MotorController.cpp:112, main.cpp:190.)
      armSend("estop", "E-STOP: arm + local wheels");
      g_arm_busy = false;   // whatever it was doing, it is not doing it now
    }

    if (y && !pY) armCommand("release", "arm: unload and stow");

    pL1 = l1; pR1 = r1; pA = a; pX = x; pY = y; pB = b; pL2 = l2; pR2 = r2;
    pTL = tl; pTR = tr;

    if (g_dpad_drive) {
      // --- D-pad: 8-way translation at exactly +/-1 per axis ---
      //
      // OVERRIDES the sticks rather than summing with them, and forces omega to
      // zero. The point of this mode is an uncontaminated command: "north-east
      // at 30%" must mean vx=+1 vy=+1 w=0 and nothing else, or an asymmetry in
      // the result cannot be pinned on a corner. A stick resting just inside
      // its deadzone would otherwise ride along in every measurement.
      //
      // Diagonals are NOT scaled down here -- mixWheels normalises by the
      // largest wheel magnitude, so (1,1) comes out as a true 45 degrees with
      // the fastest wheel at full command, same as a cardinal direction.
      const float dvx = (dpad & DPAD_UP) ? 1.0f : (dpad & DPAD_DOWN) ? -1.0f : 0.0f;
      const float dvy = (dpad & DPAD_RIGHT) ? 1.0f : (dpad & DPAD_LEFT) ? -1.0f : 0.0f;
      if (dvx != 0.0f || dvy != 0.0f) {
        vx = dvx;
        vy = dvy;
        w = 0.0f;
      }
    }
    // The old ARM JOG mode is gone with the UART: the arm's per-axis jog
    // characters are not part of the four-command ESP-NOW protocol, so the D-pad
    // is drive-only now and g_dpad_drive has nothing to toggle between.
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

  // ToF stream at 5 Hz. Polls only when the dock tick is not already polling,
  // so the two never interleave reads on the same I2C bus -- and so a streamed
  // line during a dock run is the very frame the state machine just acted on.
  static uint32_t tof_t = 0;
  if (g_tof_stream && g_tof_ok && millis() - tof_t >= 200) {
    tof_t = millis();
    if (!g_docking) tofPoll();
    tofPrint();
  }

  static uint32_t last = 0;
  if (millis() - last >= 500) {
    last = millis();
    // Duty is labelled per corner. During a pure rotation all four must read
    // equal and non-zero -- that is the single line that separates "the mix is
    // wrong" from "a corner is not pulling", and unlabelled columns make it far
    // too easy to read the pair the wrong way round.
    // arm-rx counts REPLIES, and it earns its column: ESP-NOW's send callback
    // only tells you the arm's radio acked the frame, not that the arm parsed or
    // acted on it. A counter stuck at 0 while sends "succeed" is the difference
    // between a link that works and one that merely transmits.
    Console.printf("[%s]%s lim %.2f  vx%+.2f vy%+.2f w%+.2f  "
                   "FL%4u FR%4u RL%4u RR%4u  arm-rx %lu%s\n",
                   live ? "live" : "NO PAD", g_sim_aligned ? " [ALIGNED]" : "",
                   limit, vx, vy, w,
                   g_duty[0], g_duty[1], g_duty[2], g_duty[3],
                   (unsigned long)g_arm_replies,
                   g_docking ? g_sm.stateName() : g_arm_busy ? "  [ARM BUSY]" : "");
  }

  delay(10);  // yield for Bluepad32 / BTstack
}
