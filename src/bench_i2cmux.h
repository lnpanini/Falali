#pragma once
#include <Arduino.h>
#include <Wire.h>

// PCA9548A / TCA9548A 8-channel I2C switch — the two parts are register- and
// pin-compatible, so this works with either.
//
// Driven with a raw single-byte channel select: bit N enables channel N, 0x00
// disables all. No library needed.
//
// Why the bench needs one at all: four AS5600 encoders are hard-wired to 0x36
// with no address pin, and four VL53L0X all boot at 0x29. Behind a mux only one
// channel is visible at a time, so identical addresses stop colliding.
//
// The ToF could instead be re-addressed via their XSHUT pins (which is what the
// fabricated PCB does, on GPIO4-7) — but on the breadboard, mux channels cost no
// GPIO and no boot sequence, so they are the faster route.

inline constexpr uint8_t MUX_ADDR = 0x70;   // A0/A1/A2 -> GND

inline bool  g_mux_ok = false;
inline int8_t g_mux_cur = -2;               // -2 = unknown, forces the first write

inline bool i2cPing(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

// Select one channel, or -1 for "all off". Skips the write when already there:
// with four encoders plus four ToF this removes most of the per-tick I2C traffic.
inline void muxSelect(int8_t ch) {
  if (!g_mux_ok || ch == g_mux_cur) return;
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(ch < 0 ? 0x00 : (uint8_t)(1u << ch));
  Wire.endTransmission();
  g_mux_cur = ch;
}

inline bool muxBegin(uint8_t sda, uint8_t scl, uint32_t hz = 400000) {
  Wire.begin(sda, scl);
  Wire.setClock(hz);
  g_mux_ok = i2cPing(MUX_ADDR);
  return g_mux_ok;
}

// Walk the main bus and every channel. The first thing to reach for when a
// device does not answer — it separates "absent" from "on the wrong channel".
template <typename PRINT>
inline void muxScan(PRINT& out) {
  out.println("\n--- I2C scan ---");
  muxSelect(-1);
  out.print("main bus:");
  for (uint8_t a = 0x08; a < 0x78; a++) if (i2cPing(a)) out.printf(" 0x%02X", a);
  out.println();
  if (!g_mux_ok) { out.println("no mux at 0x70"); return; }
  for (int8_t ch = 0; ch < 8; ch++) {
    muxSelect(ch);
    out.printf("  ch%d:", ch);
    bool any = false;
    for (uint8_t a = 0x08; a < 0x78; a++)
      if (a != MUX_ADDR && i2cPing(a)) { out.printf(" 0x%02X", a); any = true; }
    out.println(any ? "" : " -");
  }
  muxSelect(-1);
}
