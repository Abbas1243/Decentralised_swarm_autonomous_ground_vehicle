#include <Wire.h>
#include <VL53L0X.h>

#define SDA_PIN 21
#define SCL_PIN 22

// 74HC138 select pins
#define A0_PIN 33   // 138 pin 1
#define A1_PIN 32   // 138 pin 2
#define A2_PIN 25   // 138 pin 3

// ---- CALIBRATION ----
#define CALIBRATE     false   // true = run calibration, false = normal mode
const int CAL_DIST_MM = 100;   // true distance of your flat matte target
const int CAL_SAMPLES = 20;    // readings averaged per sensor

// offset = measured - true. Fill these from the calibration output.
int16_t OFFSET_MM[6] = {59, 10, 6, 52, 54, 43};

const uint8_t  NUM_SENSORS     = 6;
const uint32_t TIMING_BUDGET_US = 30000;
const uint16_t MAX_VALID_MM     = 1200;   // beyond this -> treated as no return (0xFFFF)

// ---- RTOS config ----
#define APP_CORE        1      // all user tasks pinned here (WiFi/system stay on core 0)
#define TOF_PRIO        1      // low: future control/encoder tasks must preempt this
#define OUT_PRIO        2
#define TOF_STACK       4096   // bytes on ESP32
#define OUT_STACK       4096
#define DEBUG_STACK     false  // true = print stack high-water marks every 5 s

struct ScanFrame {
  uint32_t seq;
  uint32_t cycle_ms;
  uint32_t t_ms[NUM_SENSORS];   // millis() when each reading finished
  uint16_t dist[NUM_SENSORS];   // mm, 0xFFFF = no valid return
};

static VL53L0X tof;
static QueueHandle_t     scanQ;
static SemaphoreHandle_t i2cMutex;

static void selectSensor(uint8_t n) {
  digitalWrite(A0_PIN, n & 1);
  digitalWrite(A1_PIN, (n >> 1) & 1);
  digitalWrite(A2_PIN, (n >> 2) & 1);
}

// raw reading, no offset applied. Caller must NOT hold i2cMutex.
static uint16_t readRaw(uint8_t n) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);

  selectSensor(7);
  delay(2);             // delay() yields to other tasks
  selectSensor(n);
  delay(10);

  uint16_t d = 0xFFFF;
  tof.setTimeout(100);
  if (tof.init()) {
    tof.setMeasurementTimingBudget(TIMING_BUDGET_US);
    tof.readRangeSingleMillimeters();            // discard first reading
    uint16_t r = tof.readRangeSingleMillimeters();
    if (!tof.timeoutOccurred()) d = r;
  }

  xSemaphoreGive(i2cMutex);
  return d;
}

// reading with offset applied
static uint16_t readSensor(uint8_t n) {
  uint16_t d = readRaw(n);
  if (d == 0xFFFF) return d;
  int32_t c = (int32_t)d - OFFSET_MM[n];
  if (c < 0) c = 0;
  if (c > MAX_VALID_MM) return 0xFFFF;
  return (uint16_t)c;
}

static void runCalibration() {
  Serial.printf("CALIBRATION: place flat matte target at %d mm in front of ALL sensors\n", CAL_DIST_MM);
  Serial.println("Starting in 5 s...");
  delay(5000);

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint32_t sum = 0;
    uint16_t good = 0;
    for (uint16_t k = 0; k < CAL_SAMPLES; k++) {
      uint16_t d = readRaw(i);
      if (d != 0xFFFF && d < 2000) { sum += d; good++; }
    }
    if (good == 0) {
      Serial.printf("Sensor %u: no valid readings\n", i);
    } else {
      float avg = (float)sum / good;
      Serial.printf("Sensor %u: avg %.1f mm, offset = %d\n", i, avg,
                    (int)(avg - CAL_DIST_MM + (avg >= CAL_DIST_MM ? 0.5f : -0.5f)));
    }
  }
  Serial.println("Copy the offsets into OFFSET_MM[], set CALIBRATE false, re-upload.");
}

// ---------------- tasks ----------------

static void tofTask(void *) {
  uint32_t seq = 0;
  for (;;) {
    ScanFrame f;
    uint32_t t0 = millis();
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      f.dist[i] = readSensor(i);
      f.t_ms[i] = millis();
    }
    f.cycle_ms = millis() - t0;
    f.seq = seq++;
    xQueueOverwrite(scanQ, &f);   // newest frame wins; never blocks
    vTaskDelay(1);                // guarantee a yield each cycle
  }
}

static void outTask(void *) {
  ScanFrame f;
  uint32_t lastDbg = 0;
  for (;;) {
    if (xQueueReceive(scanQ, &f, pdMS_TO_TICKS(2000)) == pdTRUE) {
      Serial.printf("%lu,%lu", (unsigned long)f.seq, (unsigned long)f.cycle_ms);
      for (uint8_t i = 0; i < NUM_SENSORS; i++) Serial.printf(",%u", f.dist[i]);
      Serial.println();
    } else {
      Serial.println("WARN: no frame for 2 s");
    }
#if DEBUG_STACK
    if (millis() - lastDbg > 5000) {
      lastDbg = millis();
      Serial.printf("stack free (bytes): out=%u\n", uxTaskGetStackHighWaterMark(NULL));
    }
#endif
  }
}

// ---------------- Arduino entry ----------------

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  pinMode(A0_PIN, OUTPUT);
  pinMode(A1_PIN, OUTPUT);
  pinMode(A2_PIN, OUTPUT);
  selectSensor(7);
  delay(50);

  i2cMutex = xSemaphoreCreateMutex();
  scanQ    = xQueueCreate(1, sizeof(ScanFrame));
  if (!i2cMutex || !scanQ) {
    Serial.println("RTOS object creation failed");
    while (1) delay(1000);
  }

  if (CALIBRATE) {
    runCalibration();
    while (1) delay(1000);
  }

  for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    uint16_t d = readSensor(i);
    if (d == 0xFFFF) Serial.printf("Sensor %u FAILED or no return\n", i);
    else             Serial.printf("Sensor %u OK (%u mm)\n", i, d);
  }

  xTaskCreatePinnedToCore(tofTask, "tof", TOF_STACK, NULL, TOF_PRIO, NULL, APP_CORE);
  xTaskCreatePinnedToCore(outTask, "out", OUT_STACK, NULL, OUT_PRIO, NULL, APP_CORE);
}

void loop() {
  vTaskDelay(portMAX_DELAY);   // nothing to do; tasks run the show
}