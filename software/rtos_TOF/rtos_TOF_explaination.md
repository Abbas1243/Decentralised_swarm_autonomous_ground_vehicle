# RTOS-Based Six-Sensor VL53L0X System -- Code Documentation

## 1. Project overview

This program runs six VL53L0X Time-of-Flight (ToF) sensors using an
ESP32 and a hardware selection circuit built around a **74HC138 / 74F138
3-to-8 decoder**. The program uses FreeRTOS tasks to separate sensor
acquisition from serial data output.

Unlike a design that gives every sensor a separate XSHUT GPIO, this
program uses three ESP32 GPIO pins to select one of the decoder's eight
outputs. The code assumes the decoder outputs are connected through
suitable inversion (for example, an SN74HC04 inverter) to the sensors'
active-low XSHUT inputs. Under that arrangement, the selected channel's
XSHUT becomes HIGH and the other connected sensors remain LOW/in
shutdown.

The program reads the sensors one at a time. For each selected sensor,
it initializes the VL53L0X, discards the first single-shot measurement,
obtains a second measurement, applies a stored calibration offset, and
records the time at which the reading finished.

A FreeRTOS acquisition task collects a complete six-sensor frame and
places it in a one-element queue. A separate output task retrieves the
latest frame and prints it over Serial.

## 2. Main objectives

-   Operate six VL53L0X sensors while reducing the number of ESP32
    selection GPIOs.
-   Select sensor channels using a 3-to-8 decoder.
-   Reuse one VL53L0X software object because only one sensor is
    intended to be active on the I²C bus at a time.
-   Calibrate and compensate fixed distance offsets.
-   Separate sensor acquisition and serial output using FreeRTOS.
-   Protect I²C access with a mutex.
-   Transmit frame sequence, scan duration and six corrected distances.

## 3. Hardware and connections

### 3.1 Main components

-   ESP32 development board
-   6 × VL53L0X ToF sensor modules
-   74HC138 or 74F138 3-to-8 decoder
-   SN74HC04 inverter, if used to invert decoder outputs for active-high
    XSHUT release
-   Common I²C SDA and SCL bus
-   Suitable regulated power supply and common ground

### 3.2 ESP32 pin assignments in the source code

  Signal     ESP32 GPIO Purpose
  -------- ------------ -----------------------------------------------
  SDA                21 I²C data
  SCL                22 I²C clock
  A0                 33 Decoder address bit A (least significant bit)
  A1                 32 Decoder address bit B
  A2                 25 Decoder address bit C (most significant bit)

The six sensors share SDA and SCL. The decoder selection lines determine
which sensor is released from shutdown.

### 3.3 Decoder selection

The code writes a three-bit binary channel number to the decoder:

``` cpp
static void selectSensor(uint8_t n) {
  digitalWrite(A0_PIN, n & 1);
  digitalWrite(A1_PIN, (n >> 1) & 1);
  digitalWrite(A2_PIN, (n >> 2) & 1);
}
```

The selected channel is represented by the binary value of `n`.

    `n`  A2 A1 A0  Decoder output selected
  ----- ---------- -------------------------
      0    000     Y0
      1    001     Y1
      2    010     Y2
      3    011     Y3
      4    100     Y4
      5    101     Y5
      6    110     Y6
      7    111     Y7

**Electrical detail:** A 74HC138/74F138 has active-LOW outputs. The
selected Y output goes LOW; the other Y outputs remain HIGH. VL53L0X
XSHUT is also active-LOW: LOW shuts down a sensor and HIGH releases it.
Therefore, if the decoder is used to control XSHUT, the logic must be
designed so that only the selected sensor receives HIGH at XSHUT. An
inverter stage such as an SN74HC04 can provide this inversion.

With that arrangement, `selectSensor(7)` makes channel 7 the selected
decoder output. If Y7 is not connected to one of the six sensors, the
six sensor XSHUT lines remain LOW, so all six sensors are shut down.
`selectSensor(n)` for n = 0...5 then releases the corresponding sensor.

This behavior depends on the actual wiring, decoder enable-pin states,
inverter connections and pull-up/power arrangement. Verify the circuit
with a multimeter or logic probe before connecting all sensors. Do not
assume `selectSensor(7)` disables every output of the decoder; it
selects Y7.

### 3.4 Power and logic-level precautions

Check the exact sensor breakout-board and decoder datasheets. The
VL53L0X IC is a low-voltage device, and not every breakout accepts 5 V
on every pin. Ensure: - ESP32, decoder, inverter and sensor grounds are
common. - Logic levels are compatible with the supply voltage of each
IC. - XSHUT is never driven above the sensor/module's permitted
voltage. - I²C pull-ups are suitable for the bus voltage and total bus
capacitance. - The 74HC138 enable inputs are wired to the required
enabled state. - Unused decoder outputs do not accidentally release
additional sensors.

## 4. Libraries and RTOS facilities

``` cpp
#include <Wire.h>
#include <VL53L0X.h>
```

  -----------------------------------------------------------------------
  Library / facility                  Function
  ----------------------------------- -----------------------------------
  `Wire.h`                            ESP32 I²C communication.

  `VL53L0X.h`                         Initialization and
                                      distance-measurement functions for
                                      the ToF sensor.

  FreeRTOS task API                   Runs acquisition and output as
                                      separate tasks.

  FreeRTOS queue API                  Transfers complete scan frames
                                      between tasks.

  FreeRTOS semaphore API              Provides mutual exclusion for I²C
                                      access.
  -----------------------------------------------------------------------

The sketch relies on the FreeRTOS APIs supplied by the ESP32 Arduino
core. It uses a Pololu-style VL53L0X library interface.

## 5. Global configuration and constants

### 5.1 I²C pins and decoder pins

``` cpp
#define SDA_PIN 21
#define SCL_PIN 22

#define A0_PIN 33
#define A1_PIN 32
#define A2_PIN 25
```

These definitions give meaningful names to the ESP32 GPIO numbers. They
are used when starting I²C and controlling the decoder.

### 5.2 Calibration configuration

``` cpp
#define CALIBRATE     false
const int CAL_DIST_MM = 100;
const int CAL_SAMPLES = 20;

int16_t OFFSET_MM[6] = {59, 10, 6, 52, 54, 43};
```

-   `CALIBRATE` chooses between calibration mode and normal operation.
-   `CAL_DIST_MM` is the known distance from the sensor to the
    calibration target, in millimetres.
-   `CAL_SAMPLES` is the number of raw readings requested for each
    sensor during calibration.
-   `OFFSET_MM[]` stores one correction value per sensor.

The code defines the offset as:

\[ `\text{offset}`{=tex} = `\text{measured distance}`{=tex} -
`\text{true distance}`{=tex} \]

Normal-mode correction is therefore:

\[ `\text{corrected distance}`{=tex} = `\text{raw distance}`{=tex} -
`\text{offset}`{=tex} \]

For example, if a sensor reads 159 mm when the true distance is 100 mm,
its offset is +59 mm. Subtracting 59 mm from later readings compensates
for that fixed positive error, assuming the error remains sufficiently
consistent over the operating range.

The six values in the array are the calibration values currently entered
in the source code. They should be treated as the project's configured
offsets, not as proof that a new calibration was performed during every
startup.

### 5.3 Sensor and measurement configuration

``` cpp
const uint8_t  NUM_SENSORS = 6;
const uint32_t TIMING_BUDGET_US = 30000;
const uint16_t MAX_VALID_MM = 1200;
```

-   `NUM_SENSORS` sets the number of sensors to six.
-   `TIMING_BUDGET_US` sets the VL53L0X ranging timing budget to 30,000
    microseconds (30 ms).
-   `MAX_VALID_MM` defines the upper limit accepted by the correction
    function. Corrected values above 1200 mm are marked invalid.

The timing budget is the time allocated to a ranging measurement; it is
not the complete six-sensor scan period.

### 5.4 FreeRTOS configuration

``` cpp
#define APP_CORE   1
#define TOF_PRIO   1
#define OUT_PRIO   2
#define TOF_STACK  4096
#define OUT_STACK  4096
#define DEBUG_STACK false
```

  ------------------------------------------------------------------------
  Setting                                      Value Meaning
  --------------------- ---------------------------- ---------------------
  `APP_CORE`                                       1 Both application
                                                     tasks are pinned to
                                                     ESP32 core 1.

  `TOF_PRIO`                                       1 Sensor acquisition
                                                     task priority.

  `OUT_PRIO`                                       2 Serial output task
                                                     priority.

  `TOF_STACK`                                   4096 Stack allocation
                                                     requested for
                                                     acquisition task, in
                                                     bytes as intended by
                                                     this ESP32
                                                     configuration.

  `OUT_STACK`                                   4096 Stack allocation
                                                     requested for output
                                                     task.

  `DEBUG_STACK`                                false Disables periodic
                                                     stack high-water
                                                     reporting.
  ------------------------------------------------------------------------

The output task has a higher priority number than the acquisition task.
On FreeRTOS, a higher numerical priority means higher task priority.
This does not mean the output task should perform sensor acquisition;
the two tasks have separate responsibilities.

The comment that Wi-Fi/system work stays on core 0 is a design
intention, not a guarantee that every ESP32 system task is always
restricted to that core.

## 6. ScanFrame data structure

``` cpp
struct ScanFrame {
  uint32_t seq;
  uint32_t cycle_ms;
  uint32_t t_ms[NUM_SENSORS];
  uint16_t dist[NUM_SENSORS];
};
```

This structure groups all data from one six-sensor scan into a single
frame.

  -----------------------------------------------------------------------
  Field                   Type                    Meaning
  ----------------------- ----------------------- -----------------------
  `seq`                   `uint32_t`              Frame sequence number.

  `cycle_ms`              `uint32_t`              Time taken to complete
                                                  the six-sensor scan, in
                                                  milliseconds.

  `t_ms[]`                `uint32_t`              `millis()` timestamp
                                                  recorded after each
                                                  sensor reading.

  `dist[]`                `uint16_t`              Corrected distance in
                                                  millimetres; `0xFFFF`
                                                  indicates no valid
                                                  reading.
  -----------------------------------------------------------------------

The timestamps are stored in the queue frame, but the current
serial-output code does not print them. They are available for future
synchronization and sensor-fusion work.

## 7. Global objects

``` cpp
static VL53L0X tof;
static QueueHandle_t scanQ;
static SemaphoreHandle_t i2cMutex;
```

-   `tof` is one VL53L0X library object reused for each selected sensor.
    This is possible because the decoder/XSHUT hardware is intended to
    make only one sensor accessible at a time.
-   `scanQ` is a FreeRTOS queue handle used to pass a `ScanFrame` from
    the acquisition task to the output task.
-   `i2cMutex` is a mutex handle used to protect I²C-related operations.

A mutex is different from a queue: the mutex protects a shared resource,
while the queue transfers data between tasks.

## 8. Sensor selection function -- `selectSensor()`

``` cpp
static void selectSensor(uint8_t n) {
  digitalWrite(A0_PIN, n & 1);
  digitalWrite(A1_PIN, (n >> 1) & 1);
  digitalWrite(A2_PIN, (n >> 2) & 1);
}
```

This function converts the channel number into three binary output
levels.

-   `n & 1` extracts bit 0.
-   `(n >> 1) & 1` extracts bit 1.
-   `(n >> 2) & 1` extracts bit 2.

These bits drive the decoder's A0, A1 and A2 address inputs. The decoder
then selects one of its eight output channels.

The function only changes the decoder selection. It does not read a
sensor or communicate over I²C.

## 9. Raw sensor acquisition -- `readRaw()`

``` cpp
static uint16_t readRaw(uint8_t n) {
  xSemaphoreTake(i2cMutex, portMAX_DELAY);

  selectSensor(7);
  delay(2);
  selectSensor(n);
  delay(10);

  uint16_t d = 0xFFFF;
  tof.setTimeout(100);
  if (tof.init()) {
    tof.setMeasurementTimingBudget(TIMING_BUDGET_US);
    tof.readRangeSingleMillimeters();
    uint16_t r = tof.readRangeSingleMillimeters();
    if (!tof.timeoutOccurred()) d = r;
  }

  xSemaphoreGive(i2cMutex);
  return d;
}
```

This is the central hardware acquisition function. It selects one sensor
and obtains an uncorrected distance.

### 9.1 Take the I²C mutex

``` cpp
xSemaphoreTake(i2cMutex, portMAX_DELAY);
```

The task requests exclusive access to the I²C-related section.
`portMAX_DELAY` means it waits until the mutex becomes available rather
than proceeding without protection.

The function releases the mutex before returning. Its comment specifies
that the caller must not already hold this mutex, because the function
takes it internally.

### 9.2 Move to the idle/unused channel

``` cpp
selectSensor(7);
delay(2);
```

The decoder is set to channel 7 for a short interval. With the assumed
inverter wiring and with only sensor channels 0--5 connected, this is
intended to put all six connected sensors into shutdown before selecting
the next one.

Channel 7 is not a universal decoder-disable command. It is simply the
Y7 selection. The circuit must be wired so that this state leaves the
six sensor XSHUT inputs LOW.

### 9.3 Select the requested sensor

``` cpp
selectSensor(n);
delay(10);
```

The requested channel (0 through 5) is selected. Under the assumed
inverted decoder-to-XSHUT wiring, the selected sensor's XSHUT becomes
HIGH and it wakes up. The other five sensors remain shut down.

The 10 ms delay gives the sensor time to become ready before I²C
initialization is attempted. The actual required startup time should be
checked against the sensor/module behavior.

### 9.4 Initialize and configure the sensor

``` cpp
tof.setTimeout(100);
if (tof.init()) {
  tof.setMeasurementTimingBudget(TIMING_BUDGET_US);
```

-   `setTimeout(100)` configures a 100 ms timeout for supported blocking
    library operations.
-   `init()` initializes communication with the currently selected
    sensor, which is expected to respond at the default VL53L0X address
    (`0x29`).
-   The measurement timing budget is set to 30 ms.

Because only one sensor is intended to be active on the bus, all sensors
can retain the same default I²C address. The decoder/XSHUT circuit
provides hardware selection instead of assigning six different I²C
addresses.

### 9.5 Discard the first reading

``` cpp
tof.readRangeSingleMillimeters();
```

The first measurement is deliberately ignored. This can help avoid using
an initial result immediately after waking and initializing the sensor.

### 9.6 Obtain the usable reading

``` cpp
uint16_t r = tof.readRangeSingleMillimeters();
if (!tof.timeoutOccurred()) d = r;
```

A second single-shot ranging command is issued. If the library reports
no timeout, the result is copied into `d`.

The variable `d` begins as `0xFFFF`, so if initialization fails or a
timeout occurs, the function returns the invalid marker.

### 9.7 Release the mutex and return

``` cpp
xSemaphoreGive(i2cMutex);
return d;
```

The mutex is released so another task or future I²C operation can use
the protected resource. The raw measurement is returned in millimetres,
or `0xFFFF` if the function did not obtain a valid result.

**Important timing note:** This function calls `init()` and performs two
single-shot reads every time a sensor is scanned. It does not use
continuous-ranging mode. Reinitialization and the discarded first
measurement contribute to the total scan duration.

## 10. Calibration correction -- `readSensor()`

``` cpp
static uint16_t readSensor(uint8_t n) {
  uint16_t d = readRaw(n);
  if (d == 0xFFFF) return d;
  int32_t c = (int32_t)d - OFFSET_MM[n];
  if (c < 0) c = 0;
  if (c > MAX_VALID_MM) return 0xFFFF;
  return (uint16_t)c;
}
```

This function calls `readRaw()` and applies the stored offset for the
selected sensor.

### Correction equation

\[ d\_{`\text{corrected}`{=tex}} = d\_{`\text{raw}`{=tex}} -
`\text{OFFSET}`{=tex}\_{n} \]

Example for sensor 0:

-   Raw distance = 159 mm
-   Offset = 59 mm
-   Corrected distance = 100 mm

The function then applies range checks:

1.  If the raw result is `0xFFFF`, it returns the invalid marker
    immediately.
2.  If the corrected result is negative, it clamps it to zero.
3.  If the corrected result is greater than 1200 mm, it returns
    `0xFFFF`.
4.  Otherwise, it returns the corrected distance as an unsigned 16-bit
    value.

The calculation uses `int32_t` temporarily so subtracting a positive or
negative offset does not underflow as it would in an unsigned
calculation.

## 11. Calibration mode -- `runCalibration()`

Calibration mode estimates a fixed offset for each sensor using a target
placed at a known distance.

``` cpp
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
```

### 11.1 Prepare the target

The program asks the operator to place a flat matte target at 100 mm in
front of all sensors. It waits five seconds before starting.

A matte target helps reduce unwanted reflections compared with some
shiny or highly reflective surfaces. The target should be positioned
consistently relative to each sensor.

### 11.2 Collect samples

For each sensor, the program requests 20 raw measurements. It accepts a
sample only if: - It is not equal to `0xFFFF`. - It is below 2000 mm.

Valid readings are accumulated in `sum`, and the number of valid samples
is counted in `good`.

### 11.3 Calculate the average and offset

\[ `\text{average}`{=tex} =
`\frac{\text{sum of valid readings}}{\text{number of valid readings}}`{=tex}
\]

\[ `\text{offset}`{=tex} = `\text{average}`{=tex} -
`\text{known target distance}`{=tex} \]

The code prints the average and a rounded integer offset for each
sensor. If no valid readings are collected, it prints a
no-valid-readings message.

### 11.4 Apply calibration values

The calibration routine does not automatically modify `OFFSET_MM[]` or
save values to non-volatile memory. The operator must copy the printed
offsets into the source array, set `CALIBRATE` to `false`, and upload
the program again.

The calibration estimates a constant offset at one target distance. It
does not compensate for every possible source of error, such as target
reflectivity, angle, ambient light, or distance-dependent measurement
error.

## 12. FreeRTOS task architecture

The program creates two application tasks:

1.  `tofTask`: acquires six sensor readings and builds a scan frame.
2.  `outTask`: receives the latest frame and prints it over Serial.

This separation means serial formatting and transmission are not
performed inside the sensor-acquisition loop.

### Task architecture

``` text
                 ESP32
                   |
          +--------+--------+
          |                 |
       tofTask            outTask
       Priority 1         Priority 2
       Core 1             Core 1
          |                 ^
          |                 |
    Read sensors 0–5        |
          |                 |
    Build ScanFrame         |
          |                 |
          +---- scanQ ------+
             Queue length 1
```

The queue carries a copy of the complete `ScanFrame` structure from the
acquisition task to the output task.

## 13. Sensor acquisition task -- `tofTask()`

``` cpp
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
    xQueueOverwrite(scanQ, &f);
    vTaskDelay(1);
  }
}
```

### 13.1 Run continuously

`for (;;)` is an infinite task loop. It continues until the task is
deleted or the system resets.

### 13.2 Start the scan timer

``` cpp
uint32_t t0 = millis();
```

Stores the current millisecond count before the six-sensor scan begins.

### 13.3 Read each sensor

``` cpp
for (uint8_t i = 0; i < NUM_SENSORS; i++) {
  f.dist[i] = readSensor(i);
  f.t_ms[i] = millis();
}
```

The task scans sensor indices 0 through 5. Each corrected result is
stored in the frame, followed by a timestamp recorded when that reading
has finished.

The timestamps are not identical because the sensors are read
sequentially.

### 13.4 Calculate scan duration

``` cpp
f.cycle_ms = millis() - t0;
```

Calculates the total elapsed time for the complete six-sensor scan, in
milliseconds.

This is the measured scan duration. It is not necessarily 50 ms or 30
ms; the code does not define a fixed scan period. Sensor startup,
initialization, measurement operations, delays and scheduling all
contribute to the duration.

### 13.5 Assign a sequence number

``` cpp
f.seq = seq++;
```

Each completed frame receives a sequence number. The number increments
once per scan and helps downstream software identify successive frames
or detect skipped frames.

### 13.6 Send the newest frame to the queue

``` cpp
xQueueOverwrite(scanQ, &f);
```

The queue was created with a length of one. `xQueueOverwrite()` replaces
the queue's existing item with the new frame if an older frame has not
yet been consumed.

This implements a **latest-frame-wins** strategy: - The output task
receives a recent complete frame. - Old unprocessed frames are discarded
rather than building up a backlog. - The acquisition task does not wait
for the output task to print every frame.

This is useful when current sensor data is more valuable than processing
every historical frame. It also means the output task may skip sequence
numbers if it cannot keep up.

### 13.7 Yield to other tasks

``` cpp
vTaskDelay(1);
```

Blocks the acquisition task for at least one RTOS tick, allowing other
ready tasks to execute. The actual delay in milliseconds depends on the
FreeRTOS tick rate.

This is not a precise scan-period scheduler.

## 14. Serial output task -- `outTask()`

``` cpp
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
```

### 14.1 Wait for a frame

``` cpp
xQueueReceive(scanQ, &f, pdMS_TO_TICKS(2000))
```

The output task waits for a frame from `scanQ`.

-   If a frame arrives, the function copies it into `f` and returns
    `pdTRUE`.
-   If no frame arrives within the configured 2000 ms wait, it returns a
    non-success result and the warning is printed.

`pdMS_TO_TICKS(2000)` converts the requested timeout from milliseconds
into RTOS ticks.

### 14.2 Print frame metadata

``` cpp
Serial.printf("%lu,%lu", (unsigned long)f.seq, (unsigned long)f.cycle_ms);
```

The first two CSV fields are: 1. Frame sequence number. 2. Complete scan
duration in milliseconds.

The casts to `unsigned long` match the `%lu` format specifier.

### 14.3 Print six distances

``` cpp
for (uint8_t i = 0; i < NUM_SENSORS; i++)
  Serial.printf(",%u", f.dist[i]);
Serial.println();
```

The task prints the six corrected distances, each preceded by a comma,
then ends the line.

The output format is:

``` text
sequence,scan_duration_ms,sensor0_mm,sensor1_mm,sensor2_mm,sensor3_mm,sensor4_mm,sensor5_mm
```

Example format (illustrative values only):

``` text
12,684,101,98,103,100,99,102
```

If a sensor reading is invalid, its field is printed as `65535`.

### 14.4 Optional stack monitoring

When `DEBUG_STACK` is set to `true`, the task prints its stack
high-water mark every five seconds.

`uxTaskGetStackHighWaterMark(NULL)` reports the minimum amount of stack
space that has remained unused since the task started, in the units used
by the ESP32 FreeRTOS port. This helps assess whether the task stack
allocation is sufficient.

In this sketch, `DEBUG_STACK` is `false`, so this diagnostic block is
compiled out.

## 15. Arduino `setup()` function

``` cpp
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
```

### Initialization sequence

1.  Start Serial at 115200 baud.
2.  Start I²C on GPIO 21 and 22.
3.  Request a 400 kHz I²C clock.
4.  Configure the decoder address pins as outputs.
5.  Select decoder channel 7 as the initial state.
6.  Wait 50 ms for the hardware state to settle.
7.  Create the I²C mutex.
8.  Create a one-element queue large enough to hold one `ScanFrame`.
9.  If either RTOS object could not be created, print an error and stop
    normal execution.
10. If calibration mode is enabled, run calibration and then remain in
    the calibration-only state.
11. Otherwise, perform one startup test reading for each sensor and
    print its status.
12. Create the acquisition and output tasks, pinned to core 1.

### Task creation

``` cpp
xTaskCreatePinnedToCore(tofTask, "tof", TOF_STACK, NULL, TOF_PRIO, NULL, APP_CORE);
xTaskCreatePinnedToCore(outTask, "out", OUT_STACK, NULL, OUT_PRIO, NULL, APP_CORE);
```

The parameters specify: - Task function. - Task name. - Stack size. -
Optional task parameter (`NULL` here). - Task priority. - Optional task
handle (`NULL` here). - Core affinity (`APP_CORE`, which is core 1).

The sketch does not check the return values from these two task-creation
calls. For stronger fault handling, check whether each task was created
successfully and report an error if not.

## 16. Arduino `loop()` function

``` cpp
void loop() {
  vTaskDelay(portMAX_DELAY);
}
```

The Arduino loop does not perform sensor acquisition. The FreeRTOS tasks
handle the application's work.

`vTaskDelay(portMAX_DELAY)` suspends the loop task for the maximum delay
value, effectively leaving the two application tasks to run the sensor
and output functions.

## 17. Complete execution flow

``` text
ESP32 powers on
      |
      v
Start Serial and I²C
      |
      v
Configure decoder GPIO pins
      |
      v
Select channel 7 (intended idle state for sensors 0–5)
      |
      v
Create I²C mutex and one-frame queue
      |
      v
Calibration enabled?
   /       \
 Yes        No
  |          |
Run         Test sensors 0–5
calibration  once
  |          |
Stop         v
          Create tofTask and outTask
                 |
        +--------+---------+
        |                  |
     tofTask             outTask
        |                  |
 Select sensor 0           |
 Initialize sensor         |
 Discard first read        |
 Obtain second read        |
 Apply offset              |
 Store timestamp          |
        |                  |
 Repeat for sensors 1–5   |
        |                  |
 Build ScanFrame           |
        |                  |
 Overwrite scanQ ----------+
                           |
                    Receive latest frame
                           |
                    Print CSV serial line
                           |
                    Wait for next frame
```

## 18. Serial output format

The output task generates eight comma-separated fields:

    Field index Field        Unit / meaning
  ------------- ------------ ------------------------------------
              1 `seq`        Frame sequence number
              2 `cycle_ms`   Total six-sensor scan duration, ms
              3 Sensor 0     Corrected distance, mm
              4 Sensor 1     Corrected distance, mm
              5 Sensor 2     Corrected distance, mm
              6 Sensor 3     Corrected distance, mm
              7 Sensor 4     Corrected distance, mm
              8 Sensor 5     Corrected distance, mm

The `t_ms[]` timestamps are stored internally but are not included in
the current serial line.

## 19. Important timing and RTOS observations

### 19.1 This code performs single-shot ranging

Although the earlier six-sensor program used continuous ranging, this
RTOS program calls `readRangeSingleMillimeters()` twice for each
selected sensor. It discards the first result and uses the second. It
does not call `startContinuous()`.

### 19.2 Sensor scans are sequential

The program reads sensor 0, then sensor 1, and so on. A frame is a
collection of sequential measurements, not six measurements captured at
the exact same instant.

The per-sensor completion timestamps in `t_ms[]` preserve some timing
information for future sensor fusion.

### 19.3 The scan duration is measured, not fixed

`cycle_ms` records the actual elapsed time for one six-sensor pass. It
depends on sensor wake-up time, initialization, measurement time,
library timeout behavior, I²C activity and task scheduling.

The `vTaskDelay(1)` call yields after the scan; it does not make the
full scan repeat at a fixed 1 ms or 50 ms interval.

### 19.4 Higher output-task priority

`OUT_PRIO` is 2 and `TOF_PRIO` is 1. The output task has higher
priority. When a frame is available, serial formatting and transmission
can preempt the lower-priority acquisition task. This separates
responsibilities, but serial printing can still affect when the
acquisition task next runs.

### 19.5 Queue overwrite behavior

The one-element queue stores only the latest waiting frame. If the
output task is still processing an older frame when a new scan finishes,
the old waiting frame can be replaced. This prevents a growing backlog
but may cause sequence numbers to skip in the serial output.

### 19.6 Mutex scope and future tasks

The mutex currently protects the decoder switching and VL53L0X I²C
operations inside `readRaw()`. Any future task that accesses the same
I²C bus or changes the decoder selection must use the same mutex
consistently. Otherwise, two tasks could interfere with one another's
bus transactions or sensor selection.

## 20. Error handling and limitations

### Implemented

-   A 100 ms library timeout is configured for supported operations.
-   Failed initialization or a timeout results in the invalid marker
    `0xFFFF`.
-   Calibration ignores invalid raw samples and samples at or above 2000
    mm.
-   RTOS mutex and queue creation failures stop normal startup.
-   The output task reports if it receives no frame for two seconds.

### Limitations / points to verify

1.  **Decoder wiring is essential.** The code assumes the
    decoder/inverter arrangement makes only the selected sensor's XSHUT
    HIGH. Confirm the active-low decoder outputs, inverter mapping,
    enable pins and XSHUT voltage levels against the actual schematic.
2.  **Channel 7 is not a hardware disable input.** It selects Y7. It
    works as an idle state for the six sensors only if Y7 is unused by
    them and the surrounding logic keeps their XSHUT lines LOW.
3.  **No explicit recovery after a runtime failure.** A sensor that
    fails to initialize on a scan returns an invalid reading; the code
    does not perform a separate recovery sequence.
4.  **Task creation results are unchecked.** The code should check the
    return status of `xTaskCreatePinnedToCore()`.
5.  **Calibration is manual.** Printed offsets must be copied into the
    source array and uploaded again.
6.  **Offset is a constant correction.** A single-distance calibration
    cannot fully correct distance-dependent or environmental measurement
    errors.
7.  **Timestamps are not printed.** They are stored in `ScanFrame` but
    currently omitted from the CSV output.
8.  **The frame queue intentionally drops stale frames.** This is
    appropriate for latest-data applications but not for applications
    that must preserve every scan.
9.  **Serial output can influence scheduling.** The output task has
    higher priority and serial printing may delay the next
    acquisition-task execution.
10. **Stack sizes and timing need measurement.** Use stack high-water
    marks and recorded scan durations under the final workload.

## 21. Recommended future improvements

-   Verify and document the complete 74HC138 and SN74HC04 wiring,
    including enable pins and output-to-sensor mapping.
-   Print per-sensor timestamps when the Raspberry Pi or ROS 2 side
    needs measurement timing.
-   Add explicit validity flags rather than relying only on `65535`.
-   Check task-creation return values.
-   Record task stack high-water marks during development.
-   Add sensor recovery/retry logic where appropriate.
-   Measure scan duration and jitter under realistic serial and
    control-task loads.
-   Consider a dedicated I²C manager task if multiple application tasks
    will eventually access the bus.
-   Add filtering only after measuring the raw sensor noise and
    understanding the latency it introduces.

## 22. Conclusion

This program implements a six-channel ToF acquisition system using an
ESP32, a 3-to-8 decoder selection circuit and FreeRTOS. The decoder
selects one sensor at a time, allowing a single VL53L0X library object
and the shared I²C bus to be reused. Each raw reading is corrected using
a per-sensor offset and stored with a completion timestamp.

A lower-priority acquisition task builds complete scan frames, while a
higher-priority output task prints the latest available frame. A mutex
protects the sensor-selection and I²C operations, and a one-element
overwrite queue prevents old frames from accumulating.

The result is a modular starting point for integrating multi-sensor
distance measurements with later robotics processing and sensor-fusion
tasks.

------------------------------------------------------------------------

# Appendix A: Original source code

The following is the source file supplied for this documentation.

``` cpp
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
```

# Appendix B: Key functions at a glance

  -----------------------------------------------------------------------
  Function / API                      Purpose
  ----------------------------------- -----------------------------------
  `selectSensor(n)`                   Sets the decoder's three address
                                      bits.

  `readRaw(n)`                        Selects a sensor, initializes it,
                                      discards one reading and returns a
                                      raw single-shot distance.

  `readSensor(n)`                     Applies the stored calibration
                                      offset and checks the corrected
                                      range.

  `runCalibration()`                  Collects raw samples at a known
                                      distance and prints suggested
                                      offsets.

  `tofTask()`                         Scans all six sensors, timestamps
                                      readings and publishes a frame.

  `outTask()`                         Receives the newest frame and
                                      prints CSV output.

  `xSemaphoreTake()` /                Acquire and release the I²C mutex.
  `xSemaphoreGive()`                  

  `xQueueOverwrite()`                 Replaces the one queued frame with
                                      the newest frame.

  `xQueueReceive()`                   Receives a frame from the queue.

  `vTaskDelay()`                      Blocks/yields a task for a
                                      specified number of RTOS ticks.

  `xTaskCreatePinnedToCore()`         Creates a task and assigns it to a
                                      selected ESP32 core.

  `millis()`                          Returns the elapsed millisecond
                                      counter used for timestamps and
                                      scan duration.
  -----------------------------------------------------------------------
