#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <esp_mac.h>
#include <esp_wifi.h>

#include "Bts7960Motor.h"
#include "HardwareConfig.h"
#include "LimitSwitches.h"
#include "MotorController.h"
#include "TofSensorArray.h"

MotorController motors;
TofSensorArray sensors;
LimitSwitches limits;
Bts7960Motor xTravelMotor;
Bts7960Motor yTravelMotor;

constexpr size_t ESPNOW_COMMAND_MAX_LEN = 32;
char pendingWirelessCommand[ESPNOW_COMMAND_MAX_LEN] = "";
uint8_t pendingWirelessSender[6] = {0};
bool hasPendingWirelessCommand = false;
uint8_t rejectedWirelessSender[6] = {0};
bool hasRejectedWirelessSender = false;

void printMacAddress(Stream &log, const char *label, const uint8_t *mac) {
  log.print(label);
  for (int i = 0; i < 6; i++) {
    if (i > 0) {
      log.print(":");
    }
    if (mac[i] < 16) {
      log.print("0");
    }
    log.print(mac[i], HEX);
  }
  log.println();
}

void printLocalMacDiagnostics(Stream &log) {
  uint8_t staMac[6] = {0};
  esp_read_mac(staMac, ESP_MAC_WIFI_STA);

  printMacAddress(log, "Arm ESP STA MAC: ", staMac);
  log.print("WiFi.macAddress(): ");
  log.println(WiFi.macAddress());
  printMacAddress(log, "Configured arm MAC: ", ARM_ESP_MAC);
  printMacAddress(log, "Trusted trolley MAC: ", TROLLEY_ESP_MAC);
}

bool macMatches(const uint8_t *a, const uint8_t *b) {
  return memcmp(a, b, 6) == 0;
}

String serialCommand;
bool clampModeEnabled = false;
unsigned long lastSensorReadMs = 0;
bool limitTestModeEnabled = false;
unsigned long lastLimitReadMs = 0;
bool hasLastLimitStates = false;
LimitSwitchStates lastLimitStates;

struct MotorJogState {
  bool active;
  bool extending;
  unsigned long startedMs;
};

MotorJogState xJog = {false, false, 0};
MotorJogState yJog = {false, false, 0};

enum class AxisCyclePhase {
  Idle,
  Extending,
  Flipping,
  Retracting,
  Done,
  Failed,
};

enum class AxisCycleMode {
  FlipRetract,
  HomeRetract,
};

struct AxisCycleState {
  bool active;
  char axis;
  AxisCycleMode mode;
  AxisCyclePhase phase;
  unsigned long phaseStartedMs;
  unsigned long lastSensorCheckMs;
};

AxisCycleState axisCycle = {
    false, 'X', AxisCycleMode::FlipRetract, AxisCyclePhase::Idle, 0, 0};
bool runYAfterXCycle = false;

enum class RemoteWorkflow {
  None,
  Grab,
  Release,
  HomeSetup,
};

RemoteWorkflow activeRemoteWorkflow = RemoteWorkflow::None;

enum class HomeSetupPhase {
  Idle,
  RetractingX,
  RetractingY,
  Done,
  Failed,
};

struct HomeSetupState {
  bool active;
  HomeSetupPhase phase;
  unsigned long phaseStartedMs;
};

HomeSetupState homeSetup = {false, HomeSetupPhase::Idle, 0};

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  fx or x    = flip X servos");
  Serial.println("  fy or y    = flip Y servos");
  Serial.println("  ex         = extract/reset X servos home");
  Serial.println("  ey         = extract/reset Y servos home");
  Serial.println("  home or r  = reset all servos home");
  Serial.println("  tx         = test X sensors");
  Serial.println("  ty         = test Y sensors");
  Serial.println("  ts or s    = test all sensors");
  Serial.println("  limit      = keep testing limit switches");
  Serial.println("  xext       = jog X travel extend");
  Serial.println("  xret       = jog X travel retract");
  Serial.println("  yext       = jog Y travel extend");
  Serial.println("  yret       = jog Y travel retract");
  Serial.println("  mstop      = stop X/Y travel motors");
  Serial.println("  xcycle     = X extend, flip, retract");
  Serial.println("  ycycle     = Y extend, flip, retract");
  Serial.println("  cycle      = run X cycle then Y cycle");
  Serial.println("  xhome      = X extend to air, servo home, retract");
  Serial.println("  yhome      = Y extend to air, servo home, retract");
  Serial.println("  homecycle  = run X home workflow then Y");
  Serial.println("  home_setup = servos home, retract X/Y to min");
  Serial.println("  clamp or c = keep reading sensors and clamp valid axes");
  Serial.println("  unclamp or u = return all servos home slowly");
  Serial.println("  stop       = stop active testing modes");
  Serial.println("  help or h  = show this menu");
  Serial.println();
}

void addEspNowPeerIfNeeded(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) {
    return;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, mac, 6);
  peerInfo.channel = ESPNOW_WIFI_CHANNEL;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  esp_err_t result = esp_now_add_peer(&peerInfo);
  if (result != ESP_OK && result != ESP_ERR_ESPNOW_EXIST) {
    Serial.print("ESP-NOW add peer failed: ");
    Serial.println(result);
  }
}

void sendWirelessStatus(const char *status) {
  addEspNowPeerIfNeeded(TROLLEY_ESP_MAC);
  esp_err_t result = esp_now_send(
      TROLLEY_ESP_MAC, reinterpret_cast<const uint8_t *>(status),
      strlen(status) + 1);

  Serial.print("ESP-NOW status: ");
  Serial.println(status);
  if (result != ESP_OK) {
    Serial.print("ESP-NOW status send failed: ");
    Serial.println(result);
  }
}

bool isWirelessEstopPending() {
  return hasPendingWirelessCommand &&
         strncmp(pendingWirelessCommand, "estop", ESPNOW_COMMAND_MAX_LEN) == 0;
}

void onEspNowDataReceived(const uint8_t *mac, const uint8_t *data, int len) {
  if (!macMatches(mac, TROLLEY_ESP_MAC)) {
    memcpy(rejectedWirelessSender, mac, 6);
    hasRejectedWirelessSender = true;
    return;
  }

  size_t copyLen = min(static_cast<size_t>(len), ESPNOW_COMMAND_MAX_LEN - 1);
  memcpy(pendingWirelessCommand, data, copyLen);
  pendingWirelessCommand[copyLen] = '\0';
  memcpy(pendingWirelessSender, mac, 6);
  hasPendingWirelessCommand = true;
}

void beginEspNow() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  printLocalMacDiagnostics(Serial);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed.");
    return;
  }

  esp_now_register_recv_cb(onEspNowDataReceived);
  addEspNowPeerIfNeeded(TROLLEY_ESP_MAC);
  Serial.println("ESP-NOW receiver ready on channel 1.");
}

void stopTravelMotors() {
  xTravelMotor.stop();
  yTravelMotor.stop();
  xJog.active = false;
  yJog.active = false;
  axisCycle.active = false;
  axisCycle.phase = AxisCyclePhase::Idle;
  homeSetup.active = false;
  homeSetup.phase = HomeSetupPhase::Idle;
  runYAfterXCycle = false;
}

void emergencyStopEverything(bool sendStatus) {
  if (sendStatus && isWirelessEstopPending()) {
    noInterrupts();
    hasPendingWirelessCommand = false;
    interrupts();
  }

  clampModeEnabled = false;
  limitTestModeEnabled = false;
  stopTravelMotors();
  motors.stopAll();
  activeRemoteWorkflow = RemoteWorkflow::None;

  Serial.println("Emergency stopped: travel motors OFF, servo PWM OFF.");
  if (sendStatus) {
    sendWirelessStatus("stopped");
  }
}

void handleRejectedWirelessSender() {
  if (!hasRejectedWirelessSender) {
    return;
  }

  uint8_t senderBuffer[6];
  noInterrupts();
  memcpy(senderBuffer, rejectedWirelessSender, 6);
  hasRejectedWirelessSender = false;
  interrupts();

  printMacAddress(Serial, "ESP-NOW rejected sender: ", senderBuffer);
}

bool isXExtendBlocked(const LimitSwitchStates &states) {
  return states.xArmMaxPressed;
}

bool isXRetractBlocked(const LimitSwitchStates &states) {
  return states.xArmMinPressed || states.limitX1Pressed || states.limitX2Pressed;
}

bool isYExtendBlocked(const LimitSwitchStates &states) {
  return states.yArmMaxPressed;
}

bool isYRetractBlocked(const LimitSwitchStates &states) {
  return states.yArmMinPressed || states.limitY1Pressed || states.limitY2Pressed;
}

void startMotorJog(char axis, bool extending) {
  clampModeEnabled = false;
  axisCycle.active = false;
  axisCycle.phase = AxisCyclePhase::Idle;
  homeSetup.active = false;
  homeSetup.phase = HomeSetupPhase::Idle;
  runYAfterXCycle = false;
  activeRemoteWorkflow = RemoteWorkflow::None;

  Bts7960Motor &motor = axis == 'X' ? xTravelMotor : yTravelMotor;
  MotorJogState &jog = axis == 'X' ? xJog : yJog;

  if (!motor.isConfigured()) {
    Serial.print(axis);
    Serial.println(" motor disabled: check BTS7960 pin config.");
    return;
  }

  LimitSwitchStates states = limits.readAll();
  bool blocked = axis == 'X'
                     ? (extending ? isXExtendBlocked(states)
                                  : isXRetractBlocked(states))
                     : (extending ? isYExtendBlocked(states)
                                  : isYRetractBlocked(states));

  if (blocked) {
    Serial.print(axis);
    Serial.print(extending ? " extend" : " retract");
    Serial.println(" blocked: matching limit is already pressed.");
    motor.stop();
    jog.active = false;
    return;
  }

  if (extending) {
    motor.setDuty(BTS_MOTOR_JOG_DUTY);
    motor.extend();
  } else {
    motor.setDuty(BTS_MOTOR_JOG_DUTY);
    motor.retract();
  }

  jog.active = true;
  jog.extending = extending;
  jog.startedMs = millis();

  Serial.print(axis);
  Serial.print(extending ? " extend" : " retract");
  Serial.println(" jog started.");
}

void serviceMotorJog(char axis, Bts7960Motor &motor, MotorJogState &jog,
                     const LimitSwitchStates &states, unsigned long now) {
  if (!jog.active) {
    return;
  }

  bool limitPressed = axis == 'X'
                          ? (jog.extending ? isXExtendBlocked(states)
                                           : isXRetractBlocked(states))
                          : (jog.extending ? isYExtendBlocked(states)
                                           : isYRetractBlocked(states));

  if (limitPressed) {
    motor.stop();
    jog.active = false;
    Serial.print(axis);
    Serial.println(" jog stopped: limit switch pressed.");
    return;
  }

  if (now - jog.startedMs >= BTS_MOTOR_JOG_TIMEOUT_MS) {
    motor.stop();
    jog.active = false;
    Serial.print(axis);
    Serial.println(" jog stopped: timeout.");
  }
}

void serviceMotorJogs() {
  if (!xJog.active && !yJog.active) {
    return;
  }

  LimitSwitchStates states = limits.readAll();
  unsigned long now = millis();
  serviceMotorJog('X', xTravelMotor, xJog, states, now);
  serviceMotorJog('Y', yTravelMotor, yJog, states, now);
}

Bts7960Motor &travelMotorForAxis(char axis) {
  return axis == 'X' ? xTravelMotor : yTravelMotor;
}

MotorJogState &jogStateForAxis(char axis) {
  return axis == 'X' ? xJog : yJog;
}

bool isAxisExtendBlocked(char axis, const LimitSwitchStates &states) {
  return axis == 'X' ? isXExtendBlocked(states) : isYExtendBlocked(states);
}

bool isAxisRetractBlocked(char axis, const LimitSwitchStates &states) {
  return axis == 'X' ? isXRetractBlocked(states) : isYRetractBlocked(states);
}

bool isAxisAirDetected(char axis, const TofReadings &readings) {
  return axis == 'X'
             ? sensors.xGroupShouldFlip(readings, SENSOR_FLIP_THRESHOLD_MM)
             : sensors.yGroupShouldFlip(readings, SENSOR_FLIP_THRESHOLD_MM);
}

bool startAxisCycle(char axis, bool queueYAfterX, AxisCycleMode mode) {
  clampModeEnabled = false;
  limitTestModeEnabled = false;
  stopTravelMotors();

  Bts7960Motor &motor = travelMotorForAxis(axis);
  if (!motor.isConfigured()) {
    Serial.print(axis);
    Serial.println(" cycle rejected: motor disabled.");
    return false;
  }

  LimitSwitchStates states = limits.readAll();
  if (isAxisExtendBlocked(axis, states)) {
    Serial.print(axis);
    Serial.println(" cycle rejected: max limit already pressed.");
    return false;
  }

  axisCycle.active = true;
  axisCycle.axis = axis;
  axisCycle.mode = mode;
  axisCycle.phaseStartedMs = millis();
  axisCycle.lastSensorCheckMs = 0;
  runYAfterXCycle = queueYAfterX;

  TofReadings readings = sensors.readAll();
  if (isAxisAirDetected(axis, readings)) {
    motor.stop();
    axisCycle.phase = AxisCyclePhase::Flipping;
    Serial.print(axis);
    Serial.println(" cycle started: air already detected, waiting before servo movement.");
    return true;
  }

  motor.setDuty(BTS_MOTOR_JOG_DUTY);
  motor.extend();
  axisCycle.phase = AxisCyclePhase::Extending;

  Serial.print(axis);
  Serial.println(" cycle started: extending.");
  return true;
}

void finishAxisCycle(bool success, const char *reason) {
  Bts7960Motor &motor = travelMotorForAxis(axisCycle.axis);
  motor.stop();
  motor.setDuty(BTS_MOTOR_JOG_DUTY);

  Serial.print(axisCycle.axis);
  Serial.print(success ? " workflow done: " : " workflow failed: ");
  Serial.println(reason);

  bool shouldStartY = success && runYAfterXCycle && axisCycle.axis == 'X';
  AxisCycleMode nextMode = axisCycle.mode;
  axisCycle.active = false;
  axisCycle.phase = success ? AxisCyclePhase::Done : AxisCyclePhase::Failed;
  runYAfterXCycle = false;

  if (shouldStartY) {
    if (!startAxisCycle('Y', false, nextMode)) {
      if (activeRemoteWorkflow == RemoteWorkflow::Grab) {
        sendWirelessStatus("failed:grab");
      } else if (activeRemoteWorkflow == RemoteWorkflow::Release) {
        sendWirelessStatus("failed:release");
      }
      activeRemoteWorkflow = RemoteWorkflow::None;
    }
    return;
  }

  if (activeRemoteWorkflow == RemoteWorkflow::Grab) {
    sendWirelessStatus(success ? "done_grab" : "failed:grab");
  } else if (activeRemoteWorkflow == RemoteWorkflow::Release) {
    sendWirelessStatus(success ? "done_release" : "failed:release");
  }

  activeRemoteWorkflow = RemoteWorkflow::None;
}

void enterRetractPhase() {
  Bts7960Motor &motor = travelMotorForAxis(axisCycle.axis);
  LimitSwitchStates states = limits.readAll();

  if (isAxisRetractBlocked(axisCycle.axis, states)) {
    finishAxisCycle(true, "already at retract limit.");
    return;
  }

  motor.setDuty(BTS_MOTOR_CYCLE_RETRACT_DUTY);
  motor.retract();
  axisCycle.phase = AxisCyclePhase::Retracting;
  axisCycle.phaseStartedMs = millis();

  Serial.print(axisCycle.axis);
  Serial.println(" cycle: retracting slowly.");
}

void serviceAxisCycle() {
  if (!axisCycle.active) {
    return;
  }

  Bts7960Motor &motor = travelMotorForAxis(axisCycle.axis);
  LimitSwitchStates limitStates = limits.readAll();
  unsigned long now = millis();

  if (axisCycle.phase == AxisCyclePhase::Extending) {
    if (isAxisExtendBlocked(axisCycle.axis, limitStates)) {
      motor.stop();
      Serial.print(axisCycle.axis);
      if (axisCycle.mode == AxisCycleMode::HomeRetract) {
        finishAxisCycle(false, "max limit pressed before air detected.");
        return;
      }
      Serial.println(" cycle: extend stopped at max limit.");
      axisCycle.phase = AxisCyclePhase::Flipping;
    } else if (now - axisCycle.lastSensorCheckMs >=
               AXIS_CYCLE_SENSOR_READ_INTERVAL_MS) {
      axisCycle.lastSensorCheckMs = now;
      TofReadings readings = sensors.readAll();

      if (isAxisAirDetected(axisCycle.axis, readings)) {
        motor.stop();
        Serial.print(axisCycle.axis);
        Serial.println(" cycle: extend stopped, air detected. Waiting before servo movement.");
        axisCycle.phase = AxisCyclePhase::Flipping;
      }
    }

    if (axisCycle.phase == AxisCyclePhase::Flipping) {
      axisCycle.phaseStartedMs = millis();
      return;
    }

    if (now - axisCycle.phaseStartedMs >= BTS_MOTOR_CYCLE_TIMEOUT_MS) {
      finishAxisCycle(false, "extend timeout.");
    }
    return;
  }

  if (axisCycle.phase == AxisCyclePhase::Flipping) {
    if (now - axisCycle.phaseStartedMs < AXIS_CYCLE_SERVO_DELAY_MS) {
      return;
    }

    motor.stop();
    Serial.print(axisCycle.axis);
    Serial.println(axisCycle.mode == AxisCycleMode::HomeRetract
                       ? " workflow: returning servos home."
                       : " cycle: flipping servos.");

    if (axisCycle.axis == 'X') {
      if (axisCycle.mode == AxisCycleMode::HomeRetract) {
        if (!motors.extractXAxis(isWirelessEstopPending)) {
          emergencyStopEverything(true);
          return;
        }
      } else {
        if (!motors.flipXAxis(isWirelessEstopPending)) {
          emergencyStopEverything(true);
          return;
        }
      }
    } else {
      if (axisCycle.mode == AxisCycleMode::HomeRetract) {
        if (!motors.extractYAxis(isWirelessEstopPending)) {
          emergencyStopEverything(true);
          return;
        }
      } else {
        if (!motors.flipYAxis(isWirelessEstopPending)) {
          emergencyStopEverything(true);
          return;
        }
      }
    }

    enterRetractPhase();
    return;
  }

  if (axisCycle.phase == AxisCyclePhase::Retracting) {
    if (isAxisRetractBlocked(axisCycle.axis, limitStates)) {
      finishAxisCycle(true, "retract limit pressed.");
      return;
    }

    if (now - axisCycle.phaseStartedMs >= BTS_MOTOR_CYCLE_TIMEOUT_MS) {
      finishAxisCycle(false, "retract timeout.");
    }
  }
}

void finishHomeSetup(bool success, const char *reason) {
  xTravelMotor.stop();
  yTravelMotor.stop();
  xTravelMotor.setDuty(BTS_MOTOR_JOG_DUTY);
  yTravelMotor.setDuty(BTS_MOTOR_JOG_DUTY);

  homeSetup.active = false;
  homeSetup.phase = success ? HomeSetupPhase::Done : HomeSetupPhase::Failed;

  Serial.print(success ? "Home setup done: " : "Home setup failed: ");
  Serial.println(reason);

  if (activeRemoteWorkflow == RemoteWorkflow::HomeSetup) {
    sendWirelessStatus(success ? "done_home" : "failed:home_setup");
  }
  activeRemoteWorkflow = RemoteWorkflow::None;
}

void startHomeSetupRetractY() {
  LimitSwitchStates states = limits.readAll();
  if (isYRetractBlocked(states)) {
    finishHomeSetup(true, "Y already at min/home.");
    return;
  }

  yTravelMotor.setDuty(BTS_MOTOR_CYCLE_RETRACT_DUTY);
  yTravelMotor.retract();
  homeSetup.phase = HomeSetupPhase::RetractingY;
  homeSetup.phaseStartedMs = millis();
  Serial.println("Home setup: retracting Y slowly.");
}

void startHomeSetupRetractX() {
  LimitSwitchStates states = limits.readAll();
  if (isXRetractBlocked(states)) {
    Serial.println("Home setup: X already at min/home.");
    startHomeSetupRetractY();
    return;
  }

  xTravelMotor.setDuty(BTS_MOTOR_CYCLE_RETRACT_DUTY);
  xTravelMotor.retract();
  homeSetup.phase = HomeSetupPhase::RetractingX;
  homeSetup.phaseStartedMs = millis();
  Serial.println("Home setup: retracting X slowly.");
}

bool startHomeSetup() {
  clampModeEnabled = false;
  limitTestModeEnabled = false;
  stopTravelMotors();

  if (!xTravelMotor.isConfigured() || !yTravelMotor.isConfigured()) {
    Serial.println("Home setup rejected: travel motor config missing.");
    return false;
  }

  homeSetup.active = true;
  homeSetup.phase = HomeSetupPhase::Idle;
  homeSetup.phaseStartedMs = millis();

  Serial.println("Home setup: returning servos home slowly.");
  if (!motors.resetAllToHome(isWirelessEstopPending)) {
    emergencyStopEverything(true);
    return false;
  }

  startHomeSetupRetractX();
  return true;
}

void serviceHomeSetup() {
  if (!homeSetup.active) {
    return;
  }

  LimitSwitchStates states = limits.readAll();
  unsigned long now = millis();

  if (homeSetup.phase == HomeSetupPhase::RetractingX) {
    if (isXRetractBlocked(states)) {
      xTravelMotor.stop();
      xTravelMotor.setDuty(BTS_MOTOR_JOG_DUTY);
      Serial.println("Home setup: X min/home reached.");
      startHomeSetupRetractY();
      return;
    }

    if (now - homeSetup.phaseStartedMs >= BTS_MOTOR_CYCLE_TIMEOUT_MS) {
      finishHomeSetup(false, "X retract timeout.");
    }
    return;
  }

  if (homeSetup.phase == HomeSetupPhase::RetractingY) {
    if (isYRetractBlocked(states)) {
      finishHomeSetup(true, "Y min/home reached.");
      return;
    }

    if (now - homeSetup.phaseStartedMs >= BTS_MOTOR_CYCLE_TIMEOUT_MS) {
      finishHomeSetup(false, "Y retract timeout.");
    }
  }
}

bool limitStatesChanged(const LimitSwitchStates &current,
                        const LimitSwitchStates &previous) {
  return current.limitX1Pressed != previous.limitX1Pressed ||
         current.limitX2Pressed != previous.limitX2Pressed ||
         current.limitY1Pressed != previous.limitY1Pressed ||
         current.limitY2Pressed != previous.limitY2Pressed ||
         current.xArmMinPressed != previous.xArmMinPressed ||
         current.xArmMaxPressed != previous.xArmMaxPressed ||
         current.yArmMinPressed != previous.yArmMinPressed ||
         current.yArmMaxPressed != previous.yArmMaxPressed;
}

void printLimitSwitchStates(const LimitSwitchStates &states) {
  limits.printStates(states, Serial);
  Serial.println("------------------------------");
}

void startLimitTestMode() {
  clampModeEnabled = false;
  limitTestModeEnabled = true;
  lastLimitReadMs = 0;
  hasLastLimitStates = false;
  Serial.println("Limit switch test ON. Fast polling, printing only changes. Type stop to end.");

  lastLimitStates = limits.readAll();
  hasLastLimitStates = true;
  printLimitSwitchStates(lastLimitStates);
}

void checkClampSensors() {
  TofReadings readings = sensors.readAll();
  sensors.printReadings(readings, Serial);

  bool xShouldFlip = sensors.xGroupShouldFlip(readings, SENSOR_FLIP_THRESHOLD_MM);
  bool yShouldFlip = sensors.yGroupShouldFlip(readings, SENSOR_FLIP_THRESHOLD_MM);

  if (xShouldFlip) {
    Serial.println("X axis: CLAMP / FLIP / HOLD");
    motors.setXAxisFlipped(true);
  } else {
    Serial.println("X axis: waiting.");
  }

  if (yShouldFlip) {
    Serial.println("Y axis: CLAMP / FLIP / HOLD");
    motors.setYAxisFlipped(true);
  } else {
    Serial.println("Y axis: waiting.");
  }

  if (motors.areBothAxesFlipped()) {
    clampModeEnabled = false;
    Serial.println("Both axes clamped. Sensor reading stopped, servos holding.");
  }
}

void startClampMode() {
  limitTestModeEnabled = false;
  homeSetup.active = false;
  axisCycle.active = false;
  axisCycle.phase = AxisCyclePhase::Idle;
  activeRemoteWorkflow = RemoteWorkflow::None;
  clampModeEnabled = true;
  lastSensorReadMs = 0;
  Serial.println("Clamp mode ON. Reading sensors until stop.");
  checkClampSensors();
}

void unclampServos() {
  clampModeEnabled = false;
  limitTestModeEnabled = false;
  stopTravelMotors();
  Serial.println("Unclamping: returning all servos home slowly...");
  motors.resetAllToHome();
  Serial.println("Unclamp done. All servos home.");
}

void processCommand(String command) {
  command.trim();
  command.toLowerCase();

  if (command.length() == 0) {
    return;
  }

  if (command == "fx" || command == "x") {
    motors.flipXAxis();
    Serial.println("X axis flipped.");
  } else if (command == "fy" || command == "y") {
    motors.flipYAxis();
    Serial.println("Y axis flipped.");
  } else if (command == "ex") {
    motors.extractXAxis();
    Serial.println("X axis extracted/home.");
  } else if (command == "ey") {
    motors.extractYAxis();
    Serial.println("Y axis extracted/home.");
  } else if (command == "home" || command == "r") {
    motors.resetAllToHome();
    Serial.println("All servos reset home.");
  } else if (command == "tx") {
    TofReadings readings = sensors.readAll();
    sensors.printXReadings(readings, Serial);
  } else if (command == "ty") {
    TofReadings readings = sensors.readAll();
    sensors.printYReadings(readings, Serial);
  } else if (command == "ts" || command == "s") {
    TofReadings readings = sensors.readAll();
    sensors.printReadings(readings, Serial);
  } else if (command == "limit") {
    startLimitTestMode();
  } else if (command == "xext") {
    startMotorJog('X', true);
  } else if (command == "xret") {
    startMotorJog('X', false);
  } else if (command == "yext") {
    startMotorJog('Y', true);
  } else if (command == "yret") {
    startMotorJog('Y', false);
  } else if (command == "mstop") {
    stopTravelMotors();
    activeRemoteWorkflow = RemoteWorkflow::None;
    Serial.println("Travel motors stopped.");
  } else if (command == "xcycle") {
    startAxisCycle('X', false, AxisCycleMode::FlipRetract);
  } else if (command == "ycycle") {
    startAxisCycle('Y', false, AxisCycleMode::FlipRetract);
  } else if (command == "cycle") {
    startAxisCycle('X', true, AxisCycleMode::FlipRetract);
  } else if (command == "xhome") {
    startAxisCycle('X', false, AxisCycleMode::HomeRetract);
  } else if (command == "yhome") {
    startAxisCycle('Y', false, AxisCycleMode::HomeRetract);
  } else if (command == "homecycle") {
    startAxisCycle('X', true, AxisCycleMode::HomeRetract);
  } else if (command == "home_setup") {
    startHomeSetup();
  } else if (command == "clamp" || command == "c") {
    startClampMode();
  } else if (command == "unclamp" || command == "u") {
    unclampServos();
  } else if (command == "stop") {
    emergencyStopEverything(false);
  } else if (command == "help" || command == "h") {
    printHelp();
  } else {
    Serial.println("Invalid command.");
    printHelp();
  }
}

bool isBusyForWirelessCommand() {
  return axisCycle.active || homeSetup.active || xJog.active || yJog.active ||
         clampModeEnabled || limitTestModeEnabled;
}

void processWirelessCommand(String command) {
  command.trim();
  command.toLowerCase();

  if (command.length() == 0) {
    return;
  }

  Serial.print("ESP-NOW command: ");
  Serial.println(command);

  if (command == "estop") {
    emergencyStopEverything(true);
    return;
  }

  // mstop sits ABOVE the busy check, next to estop, and it has to.
  //
  // A running jog sets xJog.active / yJog.active, which makes
  // isBusyForWirelessCommand() true -- so a stop arriving from the remote would
  // be answered "busy" and refused, and the jog would run its full 3 s timeout
  // regardless. That silently turns a held-to-move control into a fixed burst.
  if (command == "mstop") {
    stopTravelMotors();
    activeRemoteWorkflow = RemoteWorkflow::None;
    sendWirelessStatus("stopped");
    return;
  }

  if (isBusyForWirelessCommand()) {
    sendWirelessStatus("busy");
    return;
  }

  if (command == "grab") {
    activeRemoteWorkflow = RemoteWorkflow::Grab;
    if (!startAxisCycle('X', true, AxisCycleMode::FlipRetract)) {
      activeRemoteWorkflow = RemoteWorkflow::None;
      sendWirelessStatus("rejected:grab");
    }
  } else if (command == "release") {
    activeRemoteWorkflow = RemoteWorkflow::Release;
    if (!startAxisCycle('X', true, AxisCycleMode::HomeRetract)) {
      activeRemoteWorkflow = RemoteWorkflow::None;
      sendWirelessStatus("rejected:release");
    }
  } else if (command == "home_setup") {
    activeRemoteWorkflow = RemoteWorkflow::HomeSetup;
    if (!startHomeSetup()) {
      activeRemoteWorkflow = RemoteWorkflow::None;
      sendWirelessStatus("rejected:home_setup");
    }
  } else {
    // FALL THROUGH TO THE SERIAL COMMAND SET rather than rejecting.
    //
    // processCommand() already implements the manual jogs (xext/xret/yext/yret)
    // and everything else this firmware can do. They were unreachable from the
    // radio only because the two handlers are separate functions. One line
    // exposes the lot -- and the next command added to the serial set works
    // remotely for free, instead of needing a matching string compare here.
    //
    // This does mean clamp, limit-test and the raw servo sweeps are now reachable
    // over the air. Fine between two paired boards on a bench; reconsider if this
    // ever runs somewhere a stray packet would matter.
    processCommand(command);
  }
}

void handleWirelessCommand() {
  if (!hasPendingWirelessCommand) {
    return;
  }

  char commandBuffer[ESPNOW_COMMAND_MAX_LEN];
  uint8_t senderBuffer[6];

  noInterrupts();
  strncpy(commandBuffer, pendingWirelessCommand, ESPNOW_COMMAND_MAX_LEN);
  commandBuffer[ESPNOW_COMMAND_MAX_LEN - 1] = '\0';
  memcpy(senderBuffer, pendingWirelessSender, 6);
  hasPendingWirelessCommand = false;
  interrupts();

  printMacAddress(Serial, "ESP-NOW accepted sender: ", senderBuffer);

  processWirelessCommand(String(commandBuffer));
}

void handleSerialCommand() {
  while (Serial.available()) {
    char incoming = Serial.read();

    if (incoming == '\n' || incoming == '\r') {
      processCommand(serialCommand);
      serialCommand = "";
    } else {
      serialCommand += incoming;
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  beginEspNow();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  Serial.println();
  Serial.println("==========================");
  Serial.println("ESP ARM CODE");
  Serial.println("==========================");

  motors.begin(Serial);
  sensors.begin(Serial);
  limits.begin();

  xTravelMotor.begin({BTS_X_RPWM_PIN, BTS_X_LPWM_PIN, BTS_X_R_EN_PIN,
                      BTS_X_L_EN_PIN, BTS_X_DIRECTION_INVERTED},
                     BTS_MOTOR_JOG_DUTY, Serial);
  yTravelMotor.begin({BTS_Y_RPWM_PIN, BTS_Y_LPWM_PIN, BTS_Y_R_EN_PIN,
                      BTS_Y_L_EN_PIN, BTS_Y_DIRECTION_INVERTED},
                     BTS_MOTOR_JOG_DUTY, Serial);

  Serial.println("Manual clamp system ready.");
  printHelp();
}

void loop() {
  handleRejectedWirelessSender();
  handleWirelessCommand();
  handleSerialCommand();
  serviceMotorJogs();
  serviceAxisCycle();
  serviceHomeSetup();

  unsigned long now = millis();
  if (clampModeEnabled && now - lastSensorReadMs >= SENSOR_READ_INTERVAL_MS) {
    lastSensorReadMs = now;
    checkClampSensors();
  }

  if (limitTestModeEnabled &&
      now - lastLimitReadMs >= LIMIT_SWITCH_READ_INTERVAL_MS) {
    lastLimitReadMs = now;

    LimitSwitchStates currentStates = limits.readAll();
    if (!hasLastLimitStates ||
        limitStatesChanged(currentStates, lastLimitStates)) {
      lastLimitStates = currentStates;
      hasLastLimitStates = true;
      printLimitSwitchStates(currentStates);
    }
  }
}
