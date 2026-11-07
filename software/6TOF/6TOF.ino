#include <Wire.h>
#include <VL53L0X.h>

#define SDA_PIN 21
#define SCL_PIN 22

const uint8_t NUM_SENSORS = 6;
const uint8_t XSHUT_PINS[NUM_SENSORS] = {18,19,33,32,25,16};
const uint8_t BASE_ADDR = 0x30;           // sensors get 0x30..0x35

const uint32_t TIMING_BUDGET_US = 33000;  // 33 ms (default). Lower = faster, noisier
const uint32_t CONT_PERIOD_MS   = 50;     // inter-measurement period

VL53L0X sensors[NUM_SENSORS];
uint16_t dist[NUM_SENSORS];

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // 1) Hold all sensors in reset
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }
  delay(10);

  // 2) Wake one at a time, init, assign new address, start continuous mode
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(XSHUT_PINS[i], INPUT);   // release XSHUT (board pull-up enables sensor)
    delay(10);

    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.printf("Sensor %u FAILED to init (XSHUT pin %u)\n", i, XSHUT_PINS[i]);
      while (1) delay(1000);
    }
    sensors[i].setAddress(BASE_ADDR + i);
    sensors[i].setMeasurementTimingBudget(TIMING_BUDGET_US);
    sensors[i].startContinuous(CONT_PERIOD_MS);

    Serial.printf("Sensor %u OK @ 0x%02X\n", i, BASE_ADDR + i);
  }
}

void loop() {
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    dist[i] = sensors[i].readRangeContinuousMillimeters();
    if (sensors[i].timeoutOccurred()) dist[i] = 0xFFFF;  // 65535 = timeout/error
  }

  // CSV output: d0,d1,d2,d3,d4,d5 (mm)
  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(dist[i]);
    Serial.print(i < NUM_SENSORS - 1 ? "," : "\n");
  }
}