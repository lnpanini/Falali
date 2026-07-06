// TrolleyBot — ESP32-S3 under-ride docking firmware: composition root.
//
// This is the ONLY translation unit that knows about both hardware and domain.
// It builds the real adapters, wires them into the pure domain objects, and runs
// the fixed-rate control loop. All the interesting logic lives (and is tested) in
// lib/domain; this file is just plumbing.
#include <Arduino.h>
#include <Wire.h>

#include "config.h"
#include "pins.h"

// Domain (pure)
#include "AlignmentInterpreter.h"
#include "DockingStateMachine.h"
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
AlignmentInterpreter g_interp(cfg::makeAlignConfig(), g_clock);
SafetyMonitor g_safety;
DockingStateMachine g_sm(g_drive, g_clamp, g_limits, g_safety, g_clock, cfg::makeDockConfig());

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

  g_drive.brake(false);
  g_drive.enable(true);

  if (!g_tof.begin()) g_telemetry.log("ToF init failed — check wiring/mux");
  g_telemetry.log("TrolleyBot ready. Commands: DOCK ABORT UNCLAMP STATUS");
}

void loop() {
  const uint32_t now = g_clock.millis();

  // Always service serial so ABORT is responsive.
  g_telemetry.pump();
  const Command cmd = g_telemetry.poll();
  if (cmd != Command::None) g_sm.handleCommand(cmd);

  // Fixed-rate control tick.
  if (now - g_last_control >= cfg::kControlPeriodMs) {
    g_last_control = now;
    g_limits.update();
    g_interp.update(g_tof.read());
    const AlignmentState st = g_interp.state();
    const FaultFlags faults = readFaults();
    g_safety.update(st, faults);  // refresh the gate BEFORE the state machine ticks
    g_sm.update(st);
  }

  // Periodic status publish (and immediate reply to a STATUS command).
  if (cmd == Command::Status || now - g_last_telem >= cfg::kTelemetryPeriodMs) {
    g_last_telem = now;
    g_telemetry.publish(g_sm.stateName(), g_interp.state(), readFaults());
  }
}
