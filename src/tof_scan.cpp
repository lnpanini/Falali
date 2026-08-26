// Standalone ToF bring-up test — NOT part of the docking firmware.
// Build/flash with:  pio run -e toftest -t upload   then   pio device monitor -b 115200
//
// It scans the I2C bus, then reads distance(s):
//   - directly from one VL53L0X at 0x29, OR
//   - via a TCA9548A mux at 0x70 (channels 0..3 = FL, FR, RL, RR).
// Auto-detects which you have wired, so you can start with a single sensor.
#include <Arduino.h>
#include <VL53L0X.h>
#include <Wire.h>

static const uint8_t PIN_SDA = 38;  // matches include/pins.h
static const uint8_t PIN_SCL = 39;
static const uint8_t MUX_ADDR = 0x70;
static const uint8_t N_CH = 4;
static const char* CH_NAME[N_CH] = {"FL", "FR", "RL", "RR"};

// --- Calibration knobs (override with -D at build time, then bake into config.h) ---
#ifndef TOF_TIMING_BUDGET_US
#define TOF_TIMING_BUDGET_US 200000  // 200 ms = ST "high accuracy" preset
#endif
#ifndef TOF_SCALE
#define TOF_SCALE 1.0f               // corrected = raw * SCALE + OFFSET
#endif
#ifndef TOF_OFFSET_MM
#define TOF_OFFSET_MM 0              // constant bias correction, millimetres
#endif

// Median of the last few valid readings (rejects the odd outlier).
static uint16_t medianOf(const uint16_t* src, int n) {
  uint16_t a[8];
  for (int i = 0; i < n; ++i) a[i] = src[i];
  for (int i = 1; i < n; ++i) {  // insertion sort
    uint16_t k = a[i];
    int j = i - 1;
    while (j >= 0 && a[j] > k) { a[j + 1] = a[j]; --j; }
    a[j + 1] = k;
  }
  return a[n / 2];
}

static bool g_mux = false;
static VL53L0X g_sensor[N_CH];
static VL53L0X g_single;
static bool g_single_ok = false;
static bool g_ch_ok[N_CH] = {false, false, false, false};

static void muxSelect(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(static_cast<uint8_t>(1u << ch));
  Wire.endTransmission();
}

static bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static void i2cScan() {
  Serial.println("# I2C scan:");
  uint8_t count = 0;
  for (uint8_t a = 1; a < 127; ++a) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("#   found 0x%02X\n", a);
      ++count;
    }
  }
  if (count == 0) Serial.println("#   (nothing found — check power / SDA-SCL / pull-ups)");
}

static bool initSensor(VL53L0X& s, const char* label) {
  s.setTimeout(500);
  bool ok = false;
  for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
    ok = s.init();
  }
  if (!ok) {
    Serial.printf("#   %s: VL53L0X init FAILED (after 3 tries)\n", label);
  } else {
    s.setMeasurementTimingBudget(TOF_TIMING_BUDGET_US);
    Serial.printf("#   %s: init OK\n", label);
  }
  return ok;
}

static void printReading(VL53L0X& s, bool ok, const char* label) {
  Serial.printf("%s=", label);
  if (!ok) {
    Serial.print("no-init");
    return;
  }
  const uint16_t mm = s.readRangeSingleMillimeters();
  if (s.timeoutOccurred()) Serial.print("timeout");
  else if (mm >= 8000) Serial.print("out-of-range");
  else Serial.printf("%u mm", mm);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);  // 100 kHz — more tolerant of breadboard/jumper wiring

  Serial.println("\n# Falali ToF bring-up test (v2 diagnostic)");
  Serial.printf("# SDA=GPIO%u  SCL=GPIO%u  I2C=100kHz\n", PIN_SDA, PIN_SCL);
  i2cScan();

  g_mux = i2cPresent(MUX_ADDR);
  if (g_mux) {
    Serial.println("# TCA9548A mux @0x70 detected -> multi-sensor mode");
    for (uint8_t ch = 0; ch < N_CH; ++ch) {
      muxSelect(ch);
      g_ch_ok[ch] = initSensor(g_sensor[ch], CH_NAME[ch]);
    }
  } else {
    Serial.println("# No mux -> single sensor @0x29");
    g_single_ok = initSensor(g_single, "sensor");
  }
  Serial.println("# reading (each line re-checks the bus so a mid-stream capture is enough)");
}

void loop() {
  if (g_mux) {
    const bool p70 = i2cPresent(MUX_ADDR);
    Serial.printf("0x70=%c | ", p70 ? 'Y' : 'N');
    for (uint8_t ch = 0; ch < N_CH; ++ch) {
      muxSelect(ch);
      printReading(g_sensor[ch], g_ch_ok[ch], CH_NAME[ch]);
      Serial.print("  ");
    }
    Serial.println();
    return;
  }

  // Single-sensor calibration view: raw, median-filtered, and corrected.
  static uint16_t win[5];
  static int fill = 0;

  if (!g_single_ok) {
    Serial.println("dist=no-init");
    delay(300);
    return;
  }

  const uint16_t raw = g_single.readRangeSingleMillimeters();
  if (g_single.timeoutOccurred() || raw >= 8000) {
    Serial.println("raw=out-of-range");
    delay(50);
    return;
  }

  // Push into a rolling window and take the median.
  for (int i = 4; i > 0; --i) win[i] = win[i - 1];
  win[0] = raw;
  if (fill < 5) ++fill;
  const uint16_t med = medianOf(win, fill);
  const int corr = static_cast<int>(med * (TOF_SCALE) + (TOF_OFFSET_MM) + 0.5f);

  Serial.printf("raw=%4u  med=%4u  corr=%4d mm\n", raw, med, corr);
  delay(50);
}
