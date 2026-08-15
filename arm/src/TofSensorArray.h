#pragma once

#include <Adafruit_VL53L0X.h>
#include <Arduino.h>

#include "HardwareConfig.h"

struct TofReading {
  uint16_t distanceMm;
  uint8_t rangeStatus;
  bool valid;
};

struct TofReadings {
  TofReading x1;
  TofReading x2;
  TofReading y1;
  TofReading y2;
};

class TofSensorArray {
 public:
  void begin(Stream &log);
  TofReadings readAll();
  void printReadings(const TofReadings &readings, Stream &log) const;
  void printXReadings(const TofReadings &readings, Stream &log) const;
  void printYReadings(const TofReadings &readings, Stream &log) const;
  bool xGroupShouldFlip(const TofReadings &readings, int thresholdMm) const;
  bool yGroupShouldFlip(const TofReadings &readings, int thresholdMm) const;

 private:
  Adafruit_VL53L0X tofX1_;
  Adafruit_VL53L0X tofX2_;
  Adafruit_VL53L0X tofY1_;
  Adafruit_VL53L0X tofY2_;

  void startSensor(Adafruit_VL53L0X &sensor, int xshutPin, uint8_t address,
                   const char *name, Stream &log);
  TofReading readSensor(Adafruit_VL53L0X &sensor);
  bool shouldFlipPair(const TofReading &first, const TofReading &second,
                      int thresholdMm) const;
  bool isAtLeastThreshold(const TofReading &reading, int thresholdMm) const;
  void printOneReading(const char *name, const TofReading &reading,
                       Stream &log) const;
};
