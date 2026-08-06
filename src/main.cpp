// TrolleyBot — ESP32-S3 under-ride docking firmware: composition root.
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
#include "Vl53l0xMux.h"

using namespace tb;

// ---- Hardware adapters -----------------------------------------------------
ArduinoClock g_clock;

// Four wheels. EN/BRK/ALARM are ganged, so every motor gets the same shared pins.
Bld120aMotor g_fl(pins::kWheelSV[0], pins::kWheelPwmCh[0], pins::kWheelFR[0], pins::kWheelEN, pins::kWheelBRK, pins::kWheelALARM);
Bld120aMotor g_fr(pins::kWheelSV[1], pins::kWheelPwmCh[1], pins::kWheelFR[1], pins::kWheelEN, pins::kWheelBRK, pins::kWheelALARM);
Bld120aMotor g_rl(pins::kWheelSV[2], pins::kWheelPwmCh[2], pins::kWheelFR[2], pins::kWheelEN, pins::kWheelBRK, pins::kWheelALARM);
Bld120aMotor g_rr(pins::kWheelSV[3], pins::kWheelPwmCh[3], pins::kWheelFR[3], pins::kWheelEN, pins::kWheelBRK, pins::kWheelALARM);
MecanumDrive g_drive(g_fl, g_fr, g_rl, g_rr);

Bts7960Clamp g_clamp(pins::kClampRPWM, pins::kClampRPWMCh, pins::kClampLPWM, pins::kClampLPWMCh,
                     pins::kClampEN, pins::kClampIS_Close, pins::kClampIS_Open);
GpioLimitSwitches g_limits(pins::kLimitOpen, pins::kLimitClosed);
Vl53l0xMux g_tof(cfg::kMuxAddr, cfg::kMuxChannels, cfg::kNumZones);
SerialTelemetry g_telemetry;

// ---- Domain (pure) ---------------------------------------------------------
CornerEdgeDetector g_edge(cfg::makeCornerConfig());
DeadReckonOdometry g_odom(cfg::makeOdometryCal());
SafetyMonitor g_safety;
DockingStateMachine g_sm(g_drive, g_clamp, g_limits, g_odom, g_safety, g_clock,
                         cfg::makeDockConfig());
LinkWatchdog g_link(cfg::makeLinkConfig());

uint32_t g_last_control = 0;
uint32_t g_last_telem = 0;

// Sample the hardware fault lines into the flags the safety monitor evaluates.
static FaultFlags readFaults() {
  FaultFlags f;
  f.motor_alarm = g_drive.fault();
  f.clamp_overcurrent = g_clamp.currentAmps() > cfg::kClampStallAmps;
  f.estop = digitalRead(pins::kEstop) == LOW;  // active-low button
  return f;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(pins::kI2C_SDA, pins::kI2C_SCL);
  pinMode(pins::kEstop, INPUT_PULLUP);

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

  if (!g_tof.begin()) g_telemetry.log("ToF init failed — check wiring/mux");
  g_telemetry.log("TrolleyBot ready. Commands: DOCK ABORT UNCLAMP STATUS PING RESUME");
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
  if (cmd != Command::None) g_link.feed(now);

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
      g_sm.handleCommand(cmd);
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

    bool present[cfg::kNumZones];
    for (size_t i = 0; i < cfg::kNumZones; ++i) present[i] = g_edge.present(i);

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
