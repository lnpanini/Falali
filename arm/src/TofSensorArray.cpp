#include "TofSensorArray.h"

#include <Wire.h>

void TofSensorArray::begin(Stream &log) {
  pinMode(XSHUT_X1_PIN, OUTPUT);
  pinMode(XSHUT_X2_PIN, OUTPUT);
  pinMode(XSHUT_Y1_PIN, OUTPUT);
  pinMode(XSHUT_Y2_PIN, OUTPUT);

  digitalWrite(XSHUT_X1_PIN, LOW);
  digitalWrite(XSHUT_X2_PIN, LOW);
  digitalWrite(XSHUT_Y1_PIN, LOW);
  digitalWrite(XSHUT_Y2_PIN, LOW);

  delay(50);

  startSensor(tofX1_, XSHUT_X1_PIN, TOF_X1_ADDRESS, "X1", log);
  startSensor(tofX2_, XSHUT_X2_PIN, TOF_X2_ADDRESS, "X2", log);
  startSensor(tofY1_, XSHUT_Y1_PIN, TOF_Y1_ADDRESS, "Y1", log);
  startSensor(tofY2_, XSHUT_Y2_PIN, TOF_Y2_ADDRESS, "Y2", log);

  log.println("All 4 VL53L0X sensors ready.");
}

TofReadings TofSensorArray::readAll() {
  TofReadings readings;
  readings.x1 = readSensor(tofX1_);
  readings.x2 = readSensor(tofX2_);
  readings.y1 = readSensor(tofY1_);
  readings.y2 = readSensor(tofY2_);
  return readings;
}

void TofSensorArray::printReadings(const TofReadings &readings,
                                   Stream &log) const {
  printXReadings(readings, log);
  printYReadings(readings, log);
  log.println("------------------------------");
}

void TofSensorArray::printXReadings(const TofReadings &readings,
                                    Stream &log) const {
  printOneReading("X1", readings.x1, log);
  log.print("   ");
  printOneReading("X2", readings.x2, log);
  log.print("   |   X GROUP: ");
  log.println(xGroupShouldFlip(readings, SENSOR_FLIP_THRESHOLD_MM) ? "FLIPPING"
                                                                   : "EXTRACTING");
}

void TofSensorArray::printYReadings(const TofReadings &readings,
                                    Stream &log) const {
  printOneReading("Y1", readings.y1, log);
  log.print("   ");
  printOneReading("Y2", readings.y2, log);
  log.print("   |   Y GROUP: ");
  log.println(yGroupShouldFlip(readings, SENSOR_FLIP_THRESHOLD_MM) ? "FLIPPING"
                                                                   : "EXTRACTING");
}

bool TofSensorArray::xGroupShouldFlip(const TofReadings &readings,
                                      int thresholdMm) const {
  return shouldFlipPair(readings.x1, readings.x2, thresholdMm);
}

bool TofSensorArray::yGroupShouldFlip(const TofReadings &readings,
                                      int thresholdMm) const {
  return shouldFlipPair(readings.y1, readings.y2, thresholdMm);
}

void TofSensorArray::startSensor(Adafruit_VL53L0X &sensor, int xshutPin,
                                 uint8_t address, const char *name,
                                 Stream &log) {
  digitalWrite(xshutPin, HIGH);
  delay(20);

  log.print("Starting ");
  log.print(name);
  log.print(" at address 0x");
  log.println(address, HEX);

  if (!sensor.begin(address, false, &Wire)) {
    log.print("ERROR: ");
    log.print(name);
    log.println(" not detected!");

    while (true) {
      delay(1000);
    }
  }

  log.print(name);
  log.println(" OK");
}

TofReading TofSensorArray::readSensor(Adafruit_VL53L0X &sensor) {
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);

  TofReading reading;
  reading.rangeStatus = measure.RangeStatus;
  reading.valid = measure.RangeStatus != 4;

  if (reading.valid) {
    reading.distanceMm = measure.RangeMilliMeter;
  } else {
    reading.distanceMm = TOF_OUT_OF_RANGE_MM;
  }

  return reading;
}

bool TofSensorArray::shouldFlipPair(const TofReading &first,
                                    const TofReading &second,
                                    int thresholdMm) const {
  return isAtLeastThreshold(first, thresholdMm) &&
         isAtLeastThreshold(second, thresholdMm);
}

bool TofSensorArray::isAtLeastThreshold(const TofReading &reading,
                                        int thresholdMm) const {
  return reading.distanceMm > thresholdMm;
}

void TofSensorArray::printOneReading(const char *name,
                                     const TofReading &reading,
                                     Stream &log) const {
  log.print(name);
  log.print(": ");

  if (reading.valid) {
    log.print(reading.distanceMm);
    log.print(" mm");
  } else {
    log.print(reading.distanceMm);
    log.print(" mm");
  }
}
