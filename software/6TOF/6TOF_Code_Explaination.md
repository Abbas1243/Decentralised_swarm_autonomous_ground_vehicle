# Six VL53L0X Time-of-Flight Sensors -- Code Documentation

## 1. Project overview

This program interfaces **six VL53L0X Time-of-Flight (ToF) distance
sensors** with an ESP32. All six sensors share one I²C bus. Since every
VL53L0X normally starts with the same I²C address (`0x29`), the program
uses each sensor's **XSHUT pin** to bring sensors online one at a time
and assign each a unique address.

After initialization, all six sensors operate in continuous-ranging
mode. The ESP32 reads their distance measurements and transmits one
comma-separated line over the USB serial connection.

### Main functions

1.  Initialize the ESP32 I²C peripheral.
2.  Hold all six sensors in shutdown.
3.  Wake and initialize sensors sequentially.
4.  Assign addresses `0x30` through `0x35`.
5.  Configure the measurement timing budget.
6.  Start continuous ranging on each sensor.
7.  Read all six distance values and print them as CSV.

## 2. Hardware and connections

### 2.1 Components

-   ESP32 development board
-   6 × VL53L0X ToF sensor modules
-   Common 3.3 V supply and ground connections
-   I²C wiring
-   USB cable for programming and serial monitoring

### 2.2 Pin assignments used by this program

  Signal             ESP32 GPIO
  ---------------- ------------
  I²C SDA                    21
  I²C SCL                    22
  Sensor 0 XSHUT             18
  Sensor 1 XSHUT             19
  Sensor 2 XSHUT             33
  Sensor 3 XSHUT             32
  Sensor 4 XSHUT             25
  Sensor 5 XSHUT             16

All sensors share SDA and SCL. Each sensor has its own XSHUT control
connection.

**Power and logic:** Check the exact breakout-board specifications
before wiring. The sensor IC operates at low voltage; do not assume
every breakout accepts 5 V on every pin. Ensure the ESP32 and sensor
grounds are common. The I²C bus must use suitable logic levels and
pull-ups.

## 3. Libraries

``` cpp
#include <Wire.h>
#include <VL53L0X.h>
```

  -----------------------------------------------------------------------
  Library                             Purpose
  ----------------------------------- -----------------------------------
  `Wire.h`                            Provides ESP32 I²C communication
                                      functions such as `Wire.begin()`
                                      and `Wire.setClock()`.

  `VL53L0X.h`                         Provides functions to initialize,
                                      configure and read VL53L0X sensors.
  -----------------------------------------------------------------------

The code uses the Pololu-style `VL53L0X` library API. Install a
compatible version of that library in the Arduino IDE.

## 4. Global definitions and variables

``` cpp
#define SDA_PIN 21
#define SCL_PIN 22

const uint8_t NUM_SENSORS = 6;
const uint8_t XSHUT_PINS[NUM_SENSORS] = {18,19,33,32,25,16};
const uint8_t BASE_ADDR = 0x30;

const uint32_t TIMING_BUDGET_US = 33000;
const uint32_t CONT_PERIOD_MS   = 50;

VL53L0X sensors[NUM_SENSORS];
uint16_t dist[NUM_SENSORS];
```

### Explanation

-   `SDA_PIN` and `SCL_PIN` define the ESP32 pins used for I²C data and
    clock.
-   `NUM_SENSORS` tells the program that six sensors are connected.
-   `XSHUT_PINS[]` stores the GPIO number controlling each sensor's
    shutdown input. Array index 0 corresponds to sensor 0, and so on.
-   `BASE_ADDR` is the first address assigned to a sensor.
-   `TIMING_BUDGET_US` sets the ranging measurement timing budget to
    33,000 microseconds (33 ms).
-   `CONT_PERIOD_MS` sets the requested continuous-ranging period to 50
    ms.
-   `sensors[]` contains six independent library objects. Each object
    stores the software state/configuration associated with one physical
    sensor.
-   `dist[]` stores the latest distance reading for each sensor, in
    millimetres.

### Assigned I²C addresses

    Sensor index   Assigned address
  -------------- ------------------
               0             `0x30`
               1             `0x31`
               2             `0x32`
               3             `0x33`
               4             `0x34`
               5             `0x35`

The address is calculated using `BASE_ADDR + i`.

## 5. The `setup()` function

Arduino executes `setup()` once after the ESP32 starts or resets. This
function prepares the communication bus and initializes all six sensors.

### 5.1 Start serial communication

``` cpp
Serial.begin(115200);
```

Starts serial communication at 115200 baud. The ESP32 uses this
connection to send sensor status messages and distance readings to the
Serial Monitor or another computer.

### 5.2 Start the I²C bus

``` cpp
Wire.begin(SDA_PIN, SCL_PIN);
Wire.setClock(400000);
```

-   `Wire.begin()` starts I²C using GPIO 21 as SDA and GPIO 22 as SCL.
-   `Wire.setClock(400000)` requests a 400 kHz I²C clock (Fast Mode).

I²C uses two shared signal lines: - **SDA:** carries data. - **SCL:**
carries the clock used to coordinate data transfer.

All six sensors communicate over these same two lines. Their unique
addresses allow the ESP32 to select the intended sensor.

### 5.3 Hold all sensors in shutdown

``` cpp
for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
}
delay(10);
```

This loop visits each XSHUT GPIO.

1.  `pinMode(..., OUTPUT)` configures the GPIO as an output.
2.  `digitalWrite(..., LOW)` drives the XSHUT line low.
3.  A low XSHUT level places the corresponding VL53L0X in hardware
    shutdown/reset.

**Why is this necessary?**

All VL53L0X sensors normally power up with the same default I²C address,
`0x29`. If multiple sensors are active simultaneously at that address,
the ESP32 cannot reliably communicate with them as separate devices.

Holding all sensors in shutdown ensures that no sensor responds on the
bus during the initial address-assignment process.

The 10 ms delay gives the shutdown state time to take effect.

### 5.4 Wake sensors one at a time

``` cpp
for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    pinMode(XSHUT_PINS[i], INPUT);
    delay(10);
```

The program now processes the sensors in sequence.

Changing the XSHUT GPIO to `INPUT` releases the ESP32's active drive on
that line. In this particular wiring arrangement, the sensor breakout's
pull-up is expected to bring XSHUT high and wake the sensor.

**Important:** `INPUT` does not itself drive the pin high. This approach
depends on a suitable external or breakout-board pull-up. If the board
does not provide one, the sensor may not wake reliably. Verify the
module schematic and electrical levels.

Only the current sensor is released. Sensors that have not yet been
processed remain in shutdown. Previously initialized sensors remain
active, but they have already been assigned unique addresses.

### 5.5 Configure timeout and initialize the sensor

``` cpp
    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.printf("Sensor %u FAILED to init (XSHUT pin %u)\n",
                    i, XSHUT_PINS[i]);
      while (1) delay(1000);
    }
```

-   `setTimeout(500)` sets a 500 ms timeout for supported blocking
    operations in the library.
-   `init()` attempts to communicate with and initialize the sensor.
-   If initialization fails, the program prints the sensor index and its
    XSHUT pin.
-   `while (1)` stops normal program execution, so the remaining sensors
    are not initialized.

This is a fail-stop approach: the program does not continue as if all
six sensors were available.

A timeout is a protection against waiting indefinitely during supported
library operations. It is not a guarantee that every possible I²C fault
will be recovered automatically.

### 5.6 Assign a unique I²C address

``` cpp
    sensors[i].setAddress(BASE_ADDR + i);
```

The newly awakened sensor initially responds at its default address
(`0x29`). The library sends the address-change command to that sensor,
assigning it a new address.

For example:

-   Sensor 0 receives `0x30`.
-   Sensor 1 receives `0x31`.
-   Sensor 2 receives `0x32`.

The next sensor is then awakened and changed from the default address to
its own address.

**Why does this avoid address conflicts?**

At the beginning, only one sensor is active, so the ESP32 can
communicate with the default address unambiguously. Once that sensor has
been moved to `0x30`, it no longer responds at `0x29`. The next sensor
can therefore be awakened at `0x29` and assigned `0x31`.

This process repeats until all six sensors have unique addresses.

The assigned addresses are held in the sensors' volatile configuration.
They should be assigned again after a power cycle or reset.

### 5.7 Configure measurement timing

``` cpp
    sensors[i].setMeasurementTimingBudget(TIMING_BUDGET_US);
```

Sets the ranging measurement timing budget to 33,000 microseconds (33
ms).

The timing budget is the time allocated to a ranging measurement. In
general:

-   A longer budget can improve measurement robustness or reduce noise,
    depending on conditions.
-   A shorter budget can allow faster measurements, but may reduce
    measurement quality or reliability.

The timing budget is not the same thing as the I²C clock frequency or
the continuous-ranging period.

### 5.8 Start continuous-ranging mode

``` cpp
    sensors[i].startContinuous(CONT_PERIOD_MS);
```

Starts continuous distance measurement for the current sensor.

The argument requests a 50 ms inter-measurement period. Conceptually,
the sensor repeatedly performs ranging measurements rather than
requiring the ESP32 to issue a new single-shot measurement command each
time.

The requested period and the timing budget must be compatible with the
sensor/library implementation. A 33 ms timing budget within a 50 ms
requested period leaves nominal time for the rest of the measurement
cycle, but actual timing and readout behavior should be verified
experimentally.

### 5.9 Print initialization status

``` cpp
    Serial.printf("Sensor %u OK @ 0x%02X\n", i, BASE_ADDR + i);
}
```

Prints a confirmation message for each successfully initialized sensor,
including its assigned address.

Example output:

``` text
Sensor 0 OK @ 0x30
Sensor 1 OK @ 0x31
Sensor 2 OK @ 0x32
Sensor 3 OK @ 0x33
Sensor 4 OK @ 0x34
Sensor 5 OK @ 0x35
```

If all six messages appear, the initialization sequence has completed
successfully according to the library's initialization checks.

## 6. The `loop()` function

Arduino repeatedly executes `loop()` after `setup()` finishes. In this
program, each pass reads the six sensors and prints their values.

### 6.1 Read continuous measurements

``` cpp
for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    dist[i] = sensors[i].readRangeContinuousMillimeters();
    if (sensors[i].timeoutOccurred()) dist[i] = 0xFFFF;
}
```

The loop visits each sensor object in order.

`readRangeContinuousMillimeters()` retrieves the sensor's latest
available continuous-ranging result and returns the distance in
millimetres.

The reading is stored in the matching array element:

-   `dist[0]` stores sensor 0's result.
-   `dist[1]` stores sensor 1's result.
-   ...
-   `dist[5]` stores sensor 5's result.

After the read, `timeoutOccurred()` checks whether the library recorded
a timeout for that operation. If so, the program replaces the reading
with `0xFFFF`.

`0xFFFF` is the largest unsigned 16-bit value, equal to 65,535 decimal.
It is used here as an error/timeout marker, **not as a real measured
distance**.

A reading of zero is not used as the timeout marker in this code; zero
may have a separate meaning depending on the sensor and library
behavior.

### 6.2 Print values in CSV format

``` cpp
for (uint8_t i = 0; i < NUM_SENSORS; i++) {
    Serial.print(dist[i]);
    Serial.print(i < NUM_SENSORS - 1 ? "," : "\n");
}
```

This loop prints all six stored readings on one line.

The conditional expression:

``` cpp
i < NUM_SENSORS - 1 ? "," : "\n"
```

means: - Print a comma after sensors 0 through 4. - Print a newline
after sensor 5.

Example:

``` text
523,518,530,505,512,526
```

The values represent millimetres in sensor-index order. If a timeout
occurs, the corresponding field may contain `65535`.

This is CSV-style output and can be parsed by a Python program, serial
logger, spreadsheet, or another application.

## 7. Complete program execution sequence

``` text
ESP32 powers on / resets
          |
          v
Start Serial at 115200 baud
          |
          v
Initialize I²C (SDA 21, SCL 22, 400 kHz)
          |
          v
Drive all six XSHUT pins LOW
          |
          v
Wake Sensor 0
          |
          v
Initialize Sensor 0 at default address 0x29
          |
          v
Assign address 0x30
          |
          v
Configure timing budget and start continuous mode
          |
          v
Wake Sensor 1 and assign 0x31
          |
          v
       ... repeat ...
          |
          v
Wake Sensor 5 and assign 0x35
          |
          v
       Enter loop()
          |
          v
Read Sensor 0 to Sensor 5
          |
          v
Check timeout status and mark errors
          |
          v
Print six values as one CSV line
          |
          v
Repeat continuously
```

## 8. Understanding the I²C and XSHUT relationship

XSHUT and I²C perform different jobs:

  -----------------------------------------------------------------------
  Signal/mechanism                    Function
  ----------------------------------- -----------------------------------
  XSHUT                               Enables or shuts down an individual
                                      sensor at the hardware level.

  SDA                                 Carries I²C data between the ESP32
                                      and sensors.

  SCL                                 Provides the I²C clock.

  I²C address                         Selects which active sensor is
                                      being communicated with.
  -----------------------------------------------------------------------

XSHUT does not carry distance data. It is used to control whether a
sensor is active. Once the sensor is awake and assigned a unique
address, its measurements are transferred over SDA and SCL.

The six sensors share the same physical bus, but their unique addresses
allow the ESP32 to request readings from them individually.

## 9. Important timing considerations

The program configures:

  ------------------------------------------------------------------------
  Parameter                                    Value Meaning
  --------------------- ---------------------------- ---------------------
  I²C clock                                  400 kHz Requested bus
                                                     communication speed

  Measurement timing                           33 ms Time allocated to a
  budget                                             ranging measurement

  Continuous period                            50 ms Requested interval
                                                     between continuous
                                                     measurements

  Library timeout                             500 ms Timeout configured
                                                     for supported library
                                                     operations

  Serial baud rate                            115200 Serial communication
                                                     speed
  ------------------------------------------------------------------------

### Sequential reading versus simultaneous measurement

All six sensors are placed in continuous-ranging mode during `setup()`.
They can therefore perform measurements independently of the ESP32's
sequential read loop.

However, the ESP32 retrieves the results one sensor at a time:

1.  Read sensor 0.
2.  Read sensor 1.
3.  Read sensor 2.
4.  Read sensor 3.
5.  Read sensor 4.
6.  Read sensor 5.

Consequently, the six values in one CSV line are not guaranteed to
represent measurements taken at exactly the same instant. Each read
occurs at a different point in time, and a read can wait for sensor data
depending on the library and sensor state.

The time required for a complete loop depends on sensor readiness, I²C
transfer time, library behavior and timeouts. The code does not enforce
a fixed 50 ms loop period.

For applications requiring synchronized measurements or fast sensor
fusion, measure the actual loop duration and consider non-blocking
readiness checks, timestamps, and a controlled sampling schedule.

## 10. Error handling and limitations

### Current error handling

-   Initialization failure: prints the failed sensor and XSHUT pin, then
    halts.
-   Read timeout: stores `65535` for that sensor's reading.
-   Serial output: provides a simple way to observe the six results.

### Limitations to consider

1.  **No automatic recovery:** A sensor that fails initialization stops
    the entire program.
2.  **No sensor reconnection logic:** A sensor that stops responding
    after startup is not reinitialized automatically.
3.  **No timestamps:** The CSV line does not indicate the individual
    measurement times.
4.  **Sequential readout:** The six values are read one after another,
    not as one atomic simultaneous sample.
5.  **No filtering:** The program outputs raw library readings without
    averaging or filtering.
6.  **No explicit data-validity field:** `65535` is the timeout marker,
    but downstream software must interpret it correctly.
7.  **XSHUT pull-up dependency:** Releasing a GPIO to `INPUT` assumes
    the sensor board pulls XSHUT high appropriately.

## 11. Possible future improvements

-   Add a per-sensor validity/status field to the serial output.
-   Add timestamps to support sensor fusion.
-   Measure and log the duration of each complete six-sensor scan.
-   Add retry and recovery logic for sensors that fail.
-   Add filtering, such as a moving average or median filter, if
    required by the application.
-   Use a non-blocking read strategy if predictable loop timing is
    important.
-   Validate the continuous-ranging period and actual sensor update rate
    under the chosen timing budget.
-   Add a serial data format that is easy for the Raspberry Pi or ROS 2
    node to parse.

## 12. Conclusion

The program establishes six VL53L0X ToF sensors on a shared ESP32 I²C
bus. It resolves the sensors' identical default addresses by using
individual XSHUT control lines and assigning unique addresses
sequentially. It then starts continuous ranging on all sensors, reads
their distance results in sequence, checks for timeouts, and transmits
the values in CSV format.

This provides a basic multi-sensor distance acquisition system that can
serve as the foundation for later data processing, sensor fusion, and
robotics applications.

------------------------------------------------------------------------

# Appendix A: Original source code

The following is the source file supplied for this documentation.

``` cpp
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
```

# Appendix B: Key functions at a glance

  -------------------------------------------------------------------------
  Function                              Role
  ------------------------------------- -----------------------------------
  `Serial.begin(115200)`                Starts serial communication.

  `Wire.begin(SDA_PIN, SCL_PIN)`        Starts the I²C bus.

  `Wire.setClock(400000)`               Requests a 400 kHz I²C clock.

  `pinMode(pin, OUTPUT)`                Configures an XSHUT GPIO as an
                                        output.

  `digitalWrite(pin, LOW)`              Holds a sensor in shutdown.

  `pinMode(pin, INPUT)`                 Releases the XSHUT line, relying on
                                        a pull-up to wake the sensor.

  `setTimeout(500)`                     Configures the library timeout.

  `init()`                              Initializes communication with a
                                        sensor.

  `setAddress(address)`                 Changes the sensor's I²C address.

  `setMeasurementTimingBudget(33000)`   Sets the ranging timing budget.

  `startContinuous(50)`                 Starts continuous ranging with a
                                        requested 50 ms period.

  `readRangeContinuousMillimeters()`    Reads a continuous-ranging result
                                        in millimetres.

  `timeoutOccurred()`                   Checks whether a supported
                                        operation timed out.

  `Serial.print()`                      Sends readings to the serial
                                        output.
  -------------------------------------------------------------------------
