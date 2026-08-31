// Falali — ESP32-S3 under-ride docking firmware: composition root.
//
// The ONLY translation unit that knows about both hardware and domain. It builds
// the real adapters, wires them into the pure domain objects, and runs the fixed-
// rate control loop. The interesting logic lives (and is tested) in lib/domain.
#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "pins.h"

// Domain (pure)
#include "CornerEdgeDetector.h"
#include "DeadReckonOdometry.h"
#include "DockFrame.h"
#include "DockingStateMachine.h"
#include "LinkWatchdog.h"
#include "MecanumDrive.h"
#include "SafetyMonitor.h"

// Adapters (Arduino / hardware)
#include "ArduinoClock.h"
#include "Bld120aMotor.h"
#include "Bts7960Clamp.h"
#include "GpioLimitSwitches.h"
#include "SerialTelemetry.h"
#include "Ads1115CurrentSense.h"
#include "AlignmentIndicator.h"
#include "Bno08xImu.h"
#include "Vl53l0xArray.h"

using namespace fal;

// ---- Hardware adapters -----------------------------------------------------
ArduinoClock g_clock;

// Four wheels. EN/BRK are PER-WHEEL on the fabricated board, not ganged as the
// pre-PCB design assumed — each motor gets its own pair from the netlist-derived
// table. ALARM is kNoPin: the BLD-120A exposes no fault output and the board has
// no such net, so there is no hardware fault detection (see pins.h).
Bld120aMotor g_fl(pins::kWheelSV[0], pins::kWheelPwmCh[0], pins::kWheelFR[0],
                  pins::kWheelEN[0], pins::kWheelBRK[0], pins::kWheelALARM);
Bld120aMotor g_fr(pins::kWheelSV[1], pins::kWheelPwmCh[1], pins::kWheelFR[1],
                  pins::kWheelEN[1], pins::kWheelBRK[1], pins::kWheelALARM);
Bld120aMotor g_rl(pins::kWheelSV[2], pins::kWheelPwmCh[2], pins::kWheelFR[2],
                  pins::kWheelEN[2], pins::kWheelBRK[2], pins::kWheelALARM);
Bld120aMotor g_rr(pins::kWheelSV[3], pins::kWheelPwmCh[3], pins::kWheelFR[3],
                  pins::kWheelEN[3], pins::kWheelBRK[3], pins::kWheelALARM);
MecanumDrive g_drive(g_fl, g_fr, g_rl, g_rr);

// The docking sequence CRABS IN SIDEWAYS; everything else drives nose-first.
//
// This decorator gives DockingStateMachine a rotated view of the drivetrain, so
// its "advance" is the robot's strafe. It must be paired with toDockFrame() on
// the corner booleans below -- rotate one without the other and the machine
// drives along one axis while reading edges from the perpendicular one, which
// bisects nonsense instead of failing.
//
// Only move() is rotated. stop/brake/enable/fault have no direction to speak of,
// and safeStop() deliberately still talks to g_drive directly: a decorator has
// no business sitting between an emergency stop and the motors.
class SidewaysDrive : public IDrive {
 public:
  explicit SidewaysDrive(IDrive& inner) : inner_(inner) {}
  void move(const DriveCommand& c) override {
    inner_.move(toRobotFrame(c, cfg::kDockStrafeRight));
  }
  void stop() override { inner_.stop(); }
  void enable(bool on) override { inner_.enable(on); }
  void brake(bool on) override { inner_.brake(on); }
  bool fault() const override { return inner_.fault(); }

 private:
  IDrive& inner_;
};
SidewaysDrive g_dock_drive(g_drive);

Bts7960Clamp g_clamp(pins::kClampRPWM, pins::kClampRPWMCh, pins::kClampLPWM, pins::kClampLPWMCh,
                     pins::kClampEN, pins::kClampIS_Close, pins::kClampIS_Open);
GpioLimitSwitches g_limits(pins::kLimitOpen, pins::kLimitClosed);
// ToF: XSHUT re-addressing, NOT the mux.
//
// An earlier revision used a TCA9548A mux backend on channels {0,1,2,3}; both
// the backend and the mux hardware were retired 2026-08-31. Two things were
// wrong with that arrangement on the fabricated board. The netlist wires the four VL53L0X for
// XSHUT sequencing on GPIO4-7 (pins::kTofXSHUT) with no mux involvement at all;
// and channels 0-3 of the mux are the AS5600 ENCODERS, so the ToF reads were
// aimed at the encoder branches.
//
// Confirmed on hardware 2026-08-11: releasing XSHUT one at a time and
// re-addressing to 0x30..0x33 brings up all four, and covering each sensor
// identified GPIO4=FL, GPIO5=FR, GPIO7=RR (GPIO6=RL by elimination) -- matching
// pins.h index order exactly.
Vl53l0xArray g_tof(pins::kTofXSHUT, cfg::kNumZones);
SerialTelemetry g_telemetry;

// BNO08x at 0x4B -- ADR strapped high on this breakout (bus scan 2026-08-11).
// Only 3V3/GND/SDA/SCL are wired, so reports are polled and the only recovery
// from a desync is a software reset. See Bno08xImu.h.
Bno08xImu g_imu(0x4B);

// 4x ACS758 -> ADS1115 at 0x48. Channel map verified on hardware: FL->A0,
// FR->A1, RL->A2, RR->A3, in pins.h index order.
Ads1115CurrentSense g_current(0x48);

// No LED: GPIO48 (the DevKitC RGB) is FR's ENABLE line on this board, so the
// indicator reports over telemetry.
//
// *** NOT GPIO1 OR GPIO2. *** They used to be suggested here as free, back when
// they were only the unused analog-encoder fallback. They are now the UART link
// to ESP-ARM, and an LED on the TX line would fight it. GPIO3 or GPIO14 are the
// real spares -- 3 is a strapping pin (JTAG select) and 14 is ADC2, but neither
// matters for an indicator LED.
AlignmentIndicator g_align(g_telemetry, pins::kNoPin);

// ---- Domain (pure) ---------------------------------------------------------
CornerEdgeDetector g_edge(cfg::makeCornerConfig());
DeadReckonOdometry g_odom(cfg::makeOdometryCal());
SafetyMonitor g_safety;
DockingStateMachine g_sm(g_dock_drive, g_clamp, g_limits, g_odom, g_safety, g_clock,
                         cfg::makeDockConfig());
LinkWatchdog g_link(cfg::makeLinkConfig());

uint32_t g_last_control = 0;
uint32_t g_last_telem = 0;

// Sample the hardware fault lines into the flags the safety monitor evaluates.
static FaultFlags readFaults() {
  FaultFlags f;
  f.motor_alarm = g_drive.fault();
  f.clamp_overcurrent = g_clamp.currentAmps() > cfg::kClampStallAmps;
  // The Wheel Drive PCB has no E-stop input; it belongs to the hardware loop and
  // ESP-ARM. Reporting false is honest — but it means SOFTWARE CANNOT SEE THE
  // E-STOP on this board, so the hardware loop must cut motor power directly.
  f.estop = (pins::kEstop == pins::kNoPin) ? false
                                           : (digitalRead(pins::kEstop) == LOW);
  // The only drivetrain fault signal this board actually has. motor_alarm above
  // can never assert -- there is no ALARM terminal and no such net -- and with
  // the encoders REMOVED there is no speed feedback either. Requires valid():
  // a stale reading must not be able to trip the drivetrain, nor to hide a real
  // over-current behind a frozen value.
  f.wheel_overcurrent = g_current.valid() && g_current.peakAmps() > cfg::kWheelStallAmps;
  return f;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(pins::kI2C_SDA, pins::kI2C_SCL);
  if (pins::kEstop != pins::kNoPin) pinMode(pins::kEstop, INPUT_PULLUP);

  g_fl.begin();
  g_fr.begin();
  g_rl.begin();
  g_rr.begin();
  g_clamp.begin();
  g_limits.begin();
  g_telemetry.begin();

  // Boot BRAKED and DISABLED. The drivetrain is armed only once the Pi has proved
  // it is there — LinkWatchdog starts in NEVER_SEEN, so the first control tick
  // calls safeStop() anyway; doing it here too closes the window between the
  // driver powering up and that first tick.
  g_drive.stop();
  g_drive.brake(true);
  g_drive.enable(false);

  if (!g_tof.begin()) g_telemetry.log("ToF init failed — check XSHUT wiring on GPIO4-7");
  // Same per-corner offsets the bench firmware uses, so both report distances on
  // one scale. Docking does not depend on them (see cfg::kTofOffsetMm), but a
  // number that means different things in two builds is a trap worth avoiding.
  for (size_t i = 0; i < cfg::kNumZones; ++i) g_tof.setOffset(i, cfg::kTofOffsetMm[i]);
  if (!g_imu.begin()) g_telemetry.log("BNO08x init failed at 0x4B — heading unavailable");
  if (!g_current.begin()) g_telemetry.log("ADS1115 init failed at 0x48 — NO drivetrain fault signal");
  g_align.begin();
  g_telemetry.log("Falali ready. Commands: DOCK ABORT UNCLAMP STATUS PING RESUME");
  g_telemetry.log("drivetrain disabled until the control link is up (send PING)");
}

// Everything off, right now. Called on the link-lost edge and every tick after it.
//
// Deliberately NOT Command::Abort: abort re-enables the drivetrain so an operator
// can immediately drive again, which is the opposite of what a dead link wants.
static void safeStop() {
  g_drive.stop();
  g_drive.brake(true);
  g_drive.enable(false);
  g_clamp.stop();
}

void loop() {
  const uint32_t now = g_clock.millis();

  // Always service serial so ABORT is responsive.
  g_telemetry.pump();
  const Command cmd = g_telemetry.poll();

  // ANY well-formed frame proves the Pi is alive — that is what feeds the
  // watchdog, not just PING. A stream of DOCK/STATUS keeps the link healthy too.
  //
  // EXCEPT Resume. resume() refuses unless the link is *already* carrying fresh
  // traffic; if the RESUME frame fed the watchdog first it would certify its own
  // freshness and the check could never fail. That would let a RESUME sitting in
  // the RX buffer from before a dropout clear the latch and re-enable the motors
  // on its own — precisely what the latch exists to prevent.
  if (cmd != Command::None && cmd != Command::Resume) g_link.feed(now);

  switch (cmd) {
    case Command::None:
      break;
    case Command::Heartbeat:
      break;  // the feed() above was the entire point
    case Command::Resume:
      // Refused unless the link is genuinely carrying fresh traffic.
      if (g_link.resume(now)) {
        // Force the sequence back to Idle: the Pi must re-issue DOCK, having
        // re-read the world. Silently continuing a half-finished dock against a
        // stale picture is exactly the failure the latch exists to prevent.
        g_sm.handleCommand(Command::Abort);
        g_telemetry.log("link resumed — state machine reset to IDLE, re-issue DOCK");
      } else {
        g_telemetry.log("RESUME refused — link not healthy");
      }
      break;
    default:
      // GATED. DockingStateMachine::handleCommand actuates hardware directly —
      // Abort and Dock both do drive_.brake(false) + drive_.enable(true), and
      // Abort also clears the E-stop latch. Ungated, an ABORT arriving while the
      // link was latched LOST would release the brakes and enable all four
      // drivers, which is exactly what safeStop() exists to prevent.
      if (g_link.motionAllowed()) {
        g_sm.handleCommand(cmd);
      } else {
        g_telemetry.log("command ignored — link down (send RESUME first)");
      }
      break;
  }

  // Fixed-rate control tick.
  if (now - g_last_control >= cfg::kControlPeriodMs) {
    g_last_control = now;

    if (g_link.update(now)) {  // fires once, on the transition into LOST
      g_telemetry.log("LINK LOST — braking and latching. Send RESUME to recover.");
    }

    g_limits.update();
    g_edge.update(g_tof.read());
    g_imu.update(now);
    g_current.update(now);   // one ADC channel per tick; all four every 4 ticks
    g_align.update(now, g_sm.alignmentConfirmed());

    // Rotated into the docking frame -- see SidewaysDrive above. The machine's
    // leading pair is the robot's right-hand side, because the sequence crabs
    // under the trolley rather than driving in nose-first.
    bool robot_corners[cfg::kNumZones];
    for (size_t i = 0; i < cfg::kNumZones; ++i) robot_corners[i] = g_edge.present(i);
    bool present[cfg::kNumZones];
    toDockFrame(robot_corners, present, cfg::kDockStrafeRight);

    // Refresh the gate BEFORE the state machine ticks. The SM drives odometry itself.
    g_safety.update(g_sm.alignmentConfirmed(), readFaults());

    // The watchdog outranks the state machine. While the link is down the SM is
    // not ticked at all, so it cannot command motion however it feels about it.
    if (g_link.motionAllowed()) {
      g_sm.update(present);
    } else {
      safeStop();
    }
  }

  // Periodic status publish (and an immediate reply to a STATUS command).
  if (cmd == Command::Status || now - g_last_telem >= cfg::kTelemetryPeriodMs) {
    g_last_telem = now;
    bool present[cfg::kNumZones];
    for (size_t i = 0; i < cfg::kNumZones; ++i) present[i] = g_edge.present(i);
    g_telemetry.publish(g_sm.stateName(), present, cfg::kNumZones, g_odom.pose(),
                        g_sm.alignmentConfirmed(), readFaults(), g_link.healthName());
  }
}
