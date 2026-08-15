#pragma once

#include <Arduino.h>

constexpr uint32_t SERIAL_BAUD_RATE = 115200;

constexpr int I2C_SDA_PIN = 8;
constexpr int I2C_SCL_PIN = 9;

constexpr int XSHUT_X1_PIN = 18;
constexpr int XSHUT_X2_PIN = 21;
constexpr int XSHUT_Y1_PIN = 6;
constexpr int XSHUT_Y2_PIN = 12;

constexpr uint8_t TOF_X1_ADDRESS = 0x30;
constexpr uint8_t TOF_X2_ADDRESS = 0x31;
constexpr uint8_t TOF_Y1_ADDRESS = 0x32;
constexpr uint8_t TOF_Y2_ADDRESS = 0x33;

constexpr uint8_t PCA9685_ADDRESS = 0x40;
constexpr int SERVO_COUNT = 4;
constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;
constexpr int SERVO_FREQ_HZ = 50;
constexpr int SERVO_HOME_ANGLE = 170;
constexpr int SERVO_FLIPPED_ANGLE = 80;
constexpr int SERVO_STEP_DELAY_MS = 20;

constexpr uint8_t Y_SERVO_1_CHANNEL = 0;
constexpr uint8_t Y_SERVO_2_CHANNEL = 1;
constexpr uint8_t X_SERVO_1_CHANNEL = 2;
constexpr uint8_t X_SERVO_2_CHANNEL = 3;

constexpr int SENSOR_FLIP_THRESHOLD_MM = 200;
constexpr uint16_t TOF_OUT_OF_RANGE_MM = 8191;
constexpr unsigned long SENSOR_READ_INTERVAL_MS = 1000;
constexpr unsigned long LIMIT_SWITCH_READ_INTERVAL_MS = 10;

constexpr int BTS_X_RPWM_PIN = 17;
constexpr int BTS_X_LPWM_PIN = 16;
constexpr int BTS_X_R_EN_PIN = 15;
constexpr int BTS_X_L_EN_PIN = 7;
constexpr bool BTS_X_DIRECTION_INVERTED = true;

constexpr int BTS_Y_RPWM_PIN = 38;
constexpr int BTS_Y_LPWM_PIN = 39;
constexpr int BTS_Y_R_EN_PIN = 40;
constexpr int BTS_Y_L_EN_PIN = 41;
constexpr bool BTS_Y_DIRECTION_INVERTED = false;

constexpr uint8_t BTS_MOTOR_JOG_DUTY = 120;
constexpr unsigned long BTS_MOTOR_JOG_TIMEOUT_MS = 3000;
constexpr uint8_t BTS_MOTOR_CYCLE_RETRACT_DUTY = 80;
constexpr unsigned long BTS_MOTOR_CYCLE_TIMEOUT_MS = 8000;
constexpr unsigned long AXIS_CYCLE_SENSOR_READ_INTERVAL_MS = 100;
constexpr unsigned long AXIS_CYCLE_SERVO_DELAY_MS = 500;
constexpr uint8_t ESPNOW_WIFI_CHANNEL = 1;
constexpr uint8_t ARM_ESP_MAC[6] = {0x3C, 0xDC, 0x75, 0x5C, 0x8B, 0x08};
constexpr uint8_t TROLLEY_ESP_MAC[6] = {0x14, 0xC1, 0x9F, 0x3B, 0x7B, 0xE4};
