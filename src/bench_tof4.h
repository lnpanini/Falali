#pragma once
#include <Arduino.h>
#include <VL53L0X.h>

#include "bench_i2cmux.h"

// Four upward-facing VL53L0X behind the I2C mux — the corner sensors the docking
// sequence uses to detect the trolley board's edges.
//
// Every VL53L0X boots at 0x29 and they cannot be told apart on a shared bus.
// The production PCB solves that with per-sensor XSHUT lines (GPIO4-7) and
// software re-addressing, which lib/hal_esp32/Vl53l0xArray.h implements. On the
// breadboard the mux is simpler: one channel each, no XSHUT wiring, no boot
// sequence, and XSHUT can be left floating (the breakouts pull it up).
//
// WIRING, per sensor:
//   VIN   -> 3V3          (Pololu-style boards regulate and accept 2.6-5.5 V)
//   GND   -> GND
//   SDA   -> mux SD4/5/6/7
//   SCL   -> mux SC4/5/6/7
//   XSHUT -> leave floating
//   GPIO1 -> unused
//
// Channels 0-3 are the encoders, so the ToF take 4-7.

inline constexpr uint8_t TOF_MUX_CH[4] = {4, 5, 6, 7};   // FL, FR, RL, RR
inline const char* TOF_NAME[4] = {"FL", "FR", "RL", "RR"};

inline VL53L0X g_tof[4];
inline bool     g_tof_ok[4]  = {false, false, false, false};
inline uint16_t g_tof_mm[4]  = {0, 0, 0, 0};
inline bool     g_tof_valid[4] = {false, false, false, false};

// Shorter timing budget = faster updates, slightly noisier and shorter range.
// 20 ms suits a 50 Hz control loop; the default 33 ms is more than we can use.
inline constexpr uint32_t TOF_BUDGET_US = 20000;

template <typename PRINT>
inline void tof4Begin(PRINT& out) {
  for (int i = 0; i < 4; i++) {
    muxSelect(TOF_MUX_CH[i]);
    if (!i2cPing(0x29)) {
      out.printf("  ToF %s (ch%d): not found\n", TOF_NAME[i], TOF_MUX_CH[i]);
      g_tof_ok[i] = false;
      continue;
    }
    g_tof[i].setTimeout(500);
    if (!g_tof[i].init()) {
      out.printf("  ToF %s (ch%d): found at 0x29 but init FAILED\n",
                 TOF_NAME[i], TOF_MUX_CH[i]);
      g_tof_ok[i] = false;
      continue;
    }
    g_tof[i].setMeasurementTimingBudget(TOF_BUDGET_US);
    // Continuous mode: each sensor free-runs on its own channel, so a read is
    // just "select channel, fetch latest" rather than waiting out a conversion.
    g_tof[i].startContinuous();
    g_tof_ok[i] = true;
    out.printf("  ToF %s (ch%d): ok\n", TOF_NAME[i], TOF_MUX_CH[i]);
  }
  muxSelect(-1);
}

// Read all four. Cheap enough to call every control tick.
inline void tof4Read() {
  for (int i = 0; i < 4; i++) {
    if (!g_tof_ok[i]) { g_tof_valid[i] = false; continue; }
    muxSelect(TOF_MUX_CH[i]);
    const uint16_t mm = g_tof[i].readRangeContinuousMillimeters();
    // 8190/8191 is the sensor's "no target in range" signal, not a distance.
    g_tof_valid[i] = !g_tof[i].timeoutOccurred() && mm < 8000;
    g_tof_mm[i] = mm;
  }
}

template <typename PRINT>
inline void tof4Print(PRINT& out) {
  out.print("ToF ");
  for (int i = 0; i < 4; i++) {
    if (!g_tof_ok[i])          out.printf("%s:---- ", TOF_NAME[i]);
    else if (!g_tof_valid[i])  out.printf("%s:oor  ", TOF_NAME[i]);
    else                       out.printf("%s:%4u ", TOF_NAME[i], g_tof_mm[i]);
  }
  out.println();
}
