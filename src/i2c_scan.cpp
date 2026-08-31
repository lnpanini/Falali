// I2C bus inventory for the fabricated Wheel Drive PCB.
//
// Answers two questions you cannot answer by looking at the board:
//   1. does the BNO08x respond at all on its 4-wire (3V3/GND/SDA/SCL) hookup?
//   2. what address did the PCB actually strap the ADS1115 to?
//
// PINS COME FROM include/pins.h. Nothing here is hand-typed.
//
// This matters more than it sounds. src/tof_scan.cpp still carries SDA=38,
// SCL=39 from the pre-PCB design intent, and on the fabricated board those two
// pins are FR BRK and FL BRK -- running it here drives motor brake lines as a
// bus. See the warning at the top of include/pins.h.
//
// Safe to run with 24 V ON or OFF: the only pins touched are SDA and SCL. The
// wheel control lines are never configured, so they stay in their power-on
// state, which the inverting adapters read as released.
#include <Arduino.h>
#include <Wire.h>

#include <esp_log.h>

#include "pins.h"

// What we expect to find, so the output reads as a verdict and not a puzzle.
static const char* identify(uint8_t addr) {
  switch (addr) {
    case 0x29: return "VL53L0X (still at default -- XSHUT re-addressing not run)";
    case 0x30: return "VL53L0X FL (re-addressed)";
    case 0x31: return "VL53L0X FR (re-addressed)";
    case 0x32: return "VL53L0X RL (re-addressed)";
    case 0x33: return "VL53L0X RR (re-addressed)";
    case 0x36: return "AS5600 encoder -- ABANDONED, should not be on this bus";
    case 0x48: return "ADS1115 (ADDR->GND)  <-- the wanted strap";
    case 0x49: return "ADS1115 (ADDR->VDD)";
    case 0x4A: return "ADS1115 (ADDR->SDA) *OR* BNO08x (default) -- AMBIGUOUS";
    case 0x4B: return "ADS1115 (ADDR->SCL) *OR* BNO08x (ADR high) -- AMBIGUOUS";
    case 0x70: return "TCA9548A/PCA9548A mux -- RETIRED, should not be present";
    default:   return "unknown";
  }
}

static bool ping(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static uint8_t scanRange(bool* found) {
  uint8_t n = 0;
  for (uint8_t a = 0x08; a <= 0x77; ++a) {
    const bool ok = ping(a);
    if (found) found[a] = ok;
    if (ok) {
      Serial.printf("  0x%02X  %s\n", a, identify(a));
      ++n;
    }
  }
  if (n == 0) Serial.println("  (nothing -- check 3V3, ground, and the pull-ups)");
  return n;
}

static void fullSweep();

void setup() {
  Serial.begin(115200);
  // Never let a closed terminal wedge the sketch. This cost an evening on
  // 2026-08-06 and another on 2026-08-08. See src/bench_pcb_rl.cpp.
  Serial.setTxTimeoutMs(0);
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  // A NACK is this tool ASKING whether something is there, not an error. The
  // driver logging four lines per absent address makes a 112-address sweep
  // unreadable.
  esp_log_level_set("i2c.master", ESP_LOG_NONE);

  Wire.begin(pins::kI2C_SDA, pins::kI2C_SCL, 100000);  // 100 kHz: scanning wants tolerance
  fullSweep();
}

// One flat sweep of the bus. There used to be a per-channel pass behind a
// TCA9548A mux; the mux was retired with the encoders 2026-08-31 and every
// device on this board now sits directly on the trunk.
static void fullSweep() {
  Serial.println(F("\n\n=============== I2C BUS INVENTORY ==============="));
  Serial.printf("SDA = GPIO%u   SCL = GPIO%u   100 kHz\n",
                pins::kI2C_SDA, pins::kI2C_SCL);

  bool found[0x78] = {false};
  scanRange(found);

  Serial.println(F("\n--- VERDICT ---"));
  if (!found[0x48] && !found[0x49] && !found[0x4A] && !found[0x4B])
    Serial.println(F("  NO device in 0x48-0x4B: neither ADS1115 nor BNO08x answered."));
  if (found[0x48] && found[0x4A])
    Serial.println(F("  GOOD: ADS1115 at 0x48 and BNO08x at 0x4A. No collision."));
  if (found[0x4A] && !found[0x48] && !found[0x49])
    Serial.println(F("  ONLY 0x4A responded. Either the ADS1115 is strapped to 0x4A\n"
                     "  and is COLLIDING with the BNO08x, or one of the two is dead.\n"
                     "  Two devices at one address can ACK as if healthy -- do not\n"
                     "  read this as 'both fine'. Re-strap the ADS1115 to 0x48."));
  if (found[0x36])
    Serial.println(F("  0x36 responded: an AS5600 is still wired to the bus. The encoders\n"
                     "  were abandoned -- the boards stay bolted to the motors but should\n"
                     "  not be connected to anything."));

  Serial.println(F("\nPress 'r' to repeat the sweep."));
}

void loop() {
  if (Serial.available() && Serial.read() == 'r') fullSweep();
  delay(20);
}
