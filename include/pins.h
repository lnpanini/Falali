// GPIO map for ESP32-S3-WROOM-1 N16R8.
// Validated with the gpio-config skill: 21 pins, 0 errors, 0 warnings, ~65 mA.
//
// RESERVED — never assign:
//   GPIO26–37  flash + OCTAL PSRAM (the "R8"); using 33–37 externally crashes the module
//   GPIO19/20  native USB-CDC (serial telemetry / programming)
// Strapping pins (0, 3, 45, 46) are intentionally avoided by this map.
#pragma once

#include <cstdint>

namespace pins {

// --- Wheel motors: 4× BLD120A. Order: front-left, front-right, rear-left, rear-right ---
constexpr uint8_t kWheelSV[4] = {4, 5, 6, 7};       // SV speed (LEDC PWM)
constexpr uint8_t kWheelFR[4] = {15, 16, 17, 18};   // F/R direction
constexpr uint8_t kWheelEN = 8;                     // EN, ganged for all wheels (safety cut)
constexpr uint8_t kWheelBRK = 9;                    // BRK, ganged for all wheels (safety brake)
constexpr uint8_t kWheelALARM = 10;                 // ALARM wire-OR'd (active-low), interrupt-capable

// --- Clamp actuator: BTS7960 ---
constexpr uint8_t kClampRPWM = 11;                  // close direction (LEDC PWM) — verify on hw
constexpr uint8_t kClampLPWM = 12;                  // open direction  (LEDC PWM)
constexpr uint8_t kClampEN = 13;                    // R_EN + L_EN ganged
constexpr uint8_t kClampIS_Close = 1;               // current sense, close dir (ADC1)
constexpr uint8_t kClampIS_Open = 2;                // current sense, open dir  (ADC1)

// --- Clamp travel limit switches (confirm travel ONLY; wire to GND, use pull-ups) ---
constexpr uint8_t kLimitOpen = 14;
constexpr uint8_t kLimitClosed = 21;

// --- Safety ---
constexpr uint8_t kEstop = 47;                      // active-low button (INPUT_PULLUP)

// --- Alignment ToF I²C bus (VL53L0X sensors + optional TCA9548A mux @0x70) ---
constexpr uint8_t kI2C_SDA = 38;
constexpr uint8_t kI2C_SCL = 39;

// Spare, broken out on the module, for expansion:
//   GPIO 40, 41, 42, 48 — e.g. per-sensor VL53L0X XSHUT (discrete re-addressing mode),
//   wheel FG speed feedback, or extra edge switches.

// --- LEDC PWM configuration (all PWM outputs) ---
constexpr uint32_t kPwmFreqHz = 1000;   // BLD120A SV / BTS7960 accept ~1–20 kHz
constexpr uint8_t kPwmResBits = 8;      // duty range 0..255
// LEDC channels (used on Arduino-ESP32 core 2.x; ignored by the 3.x pin-based API).
// 6 channels of 8 — one per wheel SV, two for the clamp.
constexpr uint8_t kWheelPwmCh[4] = {0, 1, 2, 3};
constexpr uint8_t kClampRPWMCh = 4;
constexpr uint8_t kClampLPWMCh = 5;

} // namespace pins
