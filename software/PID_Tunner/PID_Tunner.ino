// robot_tune.ino v4: closed-loop motion tuning firmware (ESP32, Arduino core 3.x).
// Wheel-speed PID + turn PID (gyro heading) + drive PID (distance + heading hold), 100 Hz on the robot.
// The laptop (pid_tuner.py) only sends gains/commands and receives telemetry.
// v2: saved settings (flash), Auto-cal 1-4 (polarity, deadband, scale, track), turn-translation guard.
// Syntax-checked against host stubs only, NOT compiled for the ESP32 by the assistant. No ToF here on purpose: blocking ToF reads (~33 ms) would break the 100 Hz loop.
// No secrets in chat: set WIFI_SSID / WIFI_PASS locally.

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Preferences.h>

// ---------- Network ----------
const char*    WIFI_SSID = "SSID";
const char*    WIFI_PASS = "Password";
const uint8_t  ROBOT_ID  = 1;
const uint16_t CMD_PORT  = 5005 + ROBOT_ID;   // laptop -> robot
const uint16_t TELEM_PORT = 6005;             // robot -> laptop (laptop IP learned from its HELLO packets)

// ---------- Pins (from the schematic + the team's statement of which motor is which) ----------
// Wheel WL = U2 driver (connector M2A), encoder 34/25.   Wheel WR = U5 driver (connector M1A), encoder 19/18.
// WL / WR are PHYSICAL names only. Which one the firmware treats as 'left' is decided at run time by swapSides
// (tuner buttons 'Flip front/back' and 'Swap left/right'), so fixing left/right needs no re-flash.
const int WL_1 = 23, WL_2 = 4;      // U2 inputs (schematic: IN1 = IO23, IN2 = IO4)
const int WR_1 = 2,  WR_2 = 15;     // U5 inputs (schematic: IN1 = IO2,  IN2 = IO15)
const int ENC_WL_A = 34, ENC_WL_B = 25;
const int ENC_WR_A = 19, ENC_WR_B = 18;   // A/B order assumed; if reversed the sign test says 'flip encoder'
// NOTE: 18/19 were planned as ToF XSHUT pins; pick other pins for XSHUT when the ToF array is added.
const int SDA_PIN = 21, SCL_PIN = 22;
const uint8_t MPU_ADDR = 0x68;    // 0x69 if AD0 is tied high

// ---------- Runtime parameters (changeable from the laptop; defaults are PLACEHOLDERS, measure them) ----------
float countsPerRev = 700.0f, wheelDiam = 0.034f, trackW = 0.100f;   // diameter/track measured by team; counts/rev still a PLACEHOLDER
int   minPwm = 70, maxPwm = 150;
bool  invL = false, invR = false, invGyro = false;
int8_t encSignL = 1, encSignR = 1;    // measured: both encoders read positive going forward (flags below are 'flip vs this default')
uint8_t headSrc = 1;                  // 1 = gyro (if available + calibrated), 0 = encoders
float gyroScale = 1.0f;
float scaleR = 1.0f;                  // right-wheel metres-per-count relative to the left one (from Auto-cal 3)
bool  cfgSaved = false;               // true once the settings are stored in flash
bool  swapSides = false;              // false: logical LEFT = WL (M2A, enc 34/25), logical RIGHT = WR (M1A, enc 19/18)
int   pL1 = WL_1, pL2 = WL_2, pR1 = WR_1, pR2 = WR_2;                                  // motor pins acting as logical left / right
volatile int eLA = ENC_WL_A, eLB = ENC_WL_B, eRA = ENC_WR_A, eRB = ENC_WR_B;          // encoder pins acting as logical left / right

// ---------- Fixed constants ----------
const float    D2R = 0.0174532925f;
const uint32_t CTRL_US = 10000;       // 100 Hz
const int      SPD_WIN = 5;           // speed = position change over 5 ticks (50 ms)
const float    SPEED_LPF = 0.5f;
const float    V_MIN_CMD = 0.008f;    // wheel targets below this are treated as 0
const float    W_SLEW = 6.0f;         // rad/s^2 limit on turn-rate command
const float    V_SLEW = 0.6f;         // m/s^2 limit on drive-speed command
const float    W_HOLD_MAX = 1.5f;     // rad/s limit on heading-hold correction
const float    TURN_TOL = 1.0f * D2R, TURN_RATE_TOL = 3.0f * D2R;
const float    DRIVE_TOL = 0.005f, DRIVE_V_TOL = 0.01f;
const uint32_t SETTLE_MS = 300, MOVE_TIMEOUT_MS = 15000;

// ---------- Packets (little-endian, packed) ----------
enum { C_STOP, C_TURN, C_DRIVE, C_WHEEL, C_GAINS, C_GEOM, C_LIMITS, C_ZERO, C_HEAD, C_HELLO, C_CALIB, C_REQ, C_SIGN, C_TURNG, C_ACAL, C_SAVE, C_FLIPFB, C_RSTCFG, C_FLIPR, C_SWAPLR };
struct __attribute__((packed)) Cmd   { uint8_t magic, type; float a, b, c, d; };          // 18 B, magic 0xC2
struct __attribute__((packed)) Ack   { uint8_t magic, kind; float a, b, c; };             // 14 B, magic 0xD2
struct __attribute__((packed)) Telem { uint8_t magic, state; uint16_t seq; uint32_t t_ms; // 61 B, magic 0xD1
                                       float f[12]; int16_t pwmL, pwmR; uint8_t flags; };
static_assert(sizeof(Cmd) == 18, "Cmd");
static_assert(sizeof(Ack) == 14, "Ack");
static_assert(sizeof(Telem) == 61, "Telem");

// ---------- PID (integral kept as an output-unit term so live Ki changes don't bump; anti-windup; D on measurement) ----------
struct PID {
  float kp, ki, kd, iTerm;
  PID(float p, float i, float d) : kp(p), ki(i), kd(d), iTerm(0) {}
  void reset() { iTerm = 0; }
  float update(float err, float dmeas, float dt, float lim) {
    float step = ki * err * dt;
    iTerm = constrain(iTerm + step, -lim, lim);
    float out = kp * err + iTerm - kd * dmeas;
    if ((out > lim && err > 0) || (out < -lim && err < 0)) iTerm -= step;   // saturated: undo integration
    return constrain(out, -lim, lim);
  }
};
PID pidSpeedL(3.0f, 10.0f, 0.0f), pidSpeedR(3.0f, 10.0f, 0.0f);   // err m/s -> PWM fraction
PID pidTurn(4.0f, 0.3f, 0.15f);                                    // err rad -> rad/s
PID pidDist(2.0f, 0.0f, 0.0f);                                     // err m   -> m/s
PID pidHold(3.0f, 0.0f, 0.1f);                                     // err rad -> rad/s (during drive)
PID pidDir(0.8f, 0.0f, 0.05f);                                     // gyro-only turn: err rad -> PWM fraction, D on gyro rate (rad/s)
bool turnDirect = false; float dirMax = 0.3f, turnLead = 0.05f;

// ---------- Saved settings (flash; survive a reboot). Calibration results save automatically, 'Save to robot' saves the rest ----------
PID* const pidList[5] = { &pidSpeedL, &pidTurn, &pidDist, &pidHold, &pidDir };
const uint8_t CFG_VER = 3;   // bumped: v2 settings were saved with the old motor/pin assignment and would be wrong now
struct __attribute__((packed)) Config {
  uint8_t ver; float cpr, wd, tw, scaleR, gyroScale; uint8_t minPwm, maxPwm, flags, headSrc; float g[5][3];
};
Preferences prefs;
bool saveConfig() {
  Config c;
  c.ver = CFG_VER; c.cpr = countsPerRev; c.wd = wheelDiam; c.tw = trackW; c.scaleR = scaleR; c.gyroScale = gyroScale;
  c.minPwm = (uint8_t)minPwm; c.maxPwm = (uint8_t)maxPwm; c.headSrc = headSrc;
  c.flags = (invL ? 1 : 0) | (invR ? 2 : 0) | (encSignL < 0 ? 4 : 0) | (encSignR < 0 ? 8 : 0) | (invGyro ? 16 : 0) | (swapSides ? 32 : 0);
  for (int i = 0; i < 5; i++) { c.g[i][0] = pidList[i]->kp; c.g[i][1] = pidList[i]->ki; c.g[i][2] = pidList[i]->kd; }
  size_t n = 0;
  if (prefs.begin("rtune", false)) { n = prefs.putBytes("cfg", &c, sizeof(c)); prefs.end(); }
  cfgSaved = (n == sizeof(c));
  return cfgSaved;
}
bool loadConfig() {
  Config c; bool ok = false;
  if (prefs.begin("rtune", true)) {
    if (prefs.getBytesLength("cfg") == sizeof(c)) { prefs.getBytes("cfg", &c, sizeof(c)); ok = (c.ver == CFG_VER); }
    prefs.end();
  }
  if (!ok || !(c.cpr > 10.0f) || !(c.wd > 0.005f) || !(c.tw > 0.02f) || !(c.scaleR > 0.5f && c.scaleR < 2.0f) || c.minPwm >= c.maxPwm) return false;
  countsPerRev = c.cpr; wheelDiam = c.wd; trackW = c.tw; scaleR = c.scaleR;
  gyroScale = (c.gyroScale > 0.5f && c.gyroScale < 1.5f) ? c.gyroScale : 1.0f;
  minPwm = c.minPwm; maxPwm = c.maxPwm; headSrc = c.headSrc ? 1 : 0;
  invL = c.flags & 1; invR = c.flags & 2; encSignL = (c.flags & 4) ? -1 : 1; encSignR = (c.flags & 8) ? -1 : 1; invGyro = c.flags & 16; swapSides = c.flags & 32;
  for (int i = 0; i < 5; i++) if (c.g[i][0] == c.g[i][0] && c.g[i][1] == c.g[i][1] && c.g[i][2] == c.g[i][2]) { pidList[i]->kp = c.g[i][0]; pidList[i]->ki = c.g[i][1]; pidList[i]->kd = c.g[i][2]; }
  pidSpeedR.kp = pidSpeedL.kp; pidSpeedR.ki = pidSpeedL.ki; pidSpeedR.kd = pidSpeedL.kd;
  cfgSaved = true;
  return true;
}

// ---------- State ----------
enum { ST_IDLE = 0, ST_WHEEL = 1, ST_TURN = 2, ST_DRIVE = 3, ST_CAL = 4 };
volatile int32_t encL = 0, encR = 0;
void IRAM_ATTR isrL() { encL += (digitalRead(eLA) == digitalRead(eLB)) ? 1 : -1; }
volatile uint32_t edgesLB = 0;               // diagnostic: edges seen on physical encoder WL-B (pin 25)
void IRAM_ATTR isrLB() { edgesLB++; }
void IRAM_ATTR isrR() { encR += (digitalRead(eRA) == digitalRead(eRB)) ? 1 : -1; }

WiFiUDP udp;
IPAddress tunerIP; bool haveTuner = false;
uint16_t seqNo = 0; uint32_t tickCount = 0, lastTickU = 0;

uint8_t state = ST_IDLE;
bool imuOk = false, gyroCal = false, flagDone = false, flagTimeout = false, flagFault = false, useGyroNow = false;
int imuFail = 0; float gyroBias = 0, wGyro = 0;

float x = 0, y = 0, thGyro = 0, thEnc = 0, thPrev = 0, pathS = 0;
float prevPosL = 0, prevPosR = 0, hL[SPD_WIN], hR[SPD_WIN]; int hi = 0;
float vL = 0, vR = 0, vLprev = 0, vRprev = 0, rateEncF = 0;
int16_t pwmL = 0, pwmR = 0;

float tgtL = 0, tgtR = 0;                       // wheel targets, reported in telemetry
float cmdVL = 0, cmdVR = 0; uint32_t wheelEnd = 0;
bool openLoop = false; uint32_t badSince = 0, stallSince = 0;   // open-loop bring-up mode, fault timers
float turnTarget = 0, turnStart = 0, wMax = 0.8f, wCmd = 0;
float turnPath0 = 0;                            // path length at the start of a turn (a pure spin must not translate)
bool handCap = false; int32_t capL0 = 0, capR0 = 0;   // Auto-cal 3: hand-rotation capture
bool calAbort = false; float calGyroDeg = 0;          // Auto-cal: STOP flag, raw gyro integral (no sign flip, no scale)
float distTarget = 0, pathStart = 0, vMax = 0.15f, vCmd = 0, holdTarget = 0;
uint32_t moveStart = 0, tolSince = 0;
float repTgt = 0, repErr = 0, repAux = 0;

// ---------- Motors ----------
void motorStop() {
  analogWrite(WL_1, 0); analogWrite(WL_2, 0);
  analogWrite(WR_1, 0); analogWrite(WR_2, 0);
}
void motorBrake() {                  // both inputs high = DRV8833 brake (fast stop)
  analogWrite(WL_1, 255); analogWrite(WL_2, 255);
  analogWrite(WR_1, 255); analogWrite(WR_2, 255);
}
int16_t driveSide(int in1, int in2, float u, bool invert) {   // returns signed PWM written
  if (invert) u = -u;
  u = constrain(u, -1.0f, 1.0f);
  if (fabsf(u) < 0.03f) { analogWrite(in1, 0); analogWrite(in2, 0); return 0; }
  int pwm = minPwm + (int)(fabsf(u) * (maxPwm - minPwm));
  if (u > 0) { analogWrite(in1, pwm); analogWrite(in2, 0); return pwm; }
  analogWrite(in1, 0); analogWrite(in2, pwm); return -pwm;
}

void readEnc(int32_t &l, int32_t &r) { noInterrupts(); l = encL; r = encR; interrupts(); }
void rawSide(int in1, int in2, int pwm, bool invert) {   // raw signed PWM (+ = 'positive command'), bypasses the min/max mapping
  if (invert) pwm = -pwm;
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) { analogWrite(in1, pwm); analogWrite(in2, 0); }
  else if (pwm < 0) { analogWrite(in1, 0); analogWrite(in2, -pwm); }
  else { analogWrite(in1, 0); analogWrite(in2, 0); }
}
void rawDrive(int pl, int pr) { rawSide(pL1, pL2, pl, invL); rawSide(pR1, pR2, pr, invR); }

// ---------- MPU6050 (gyro Z only) ----------
bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}
bool mpuInit() {
  Wire.begin(SDA_PIN, SCL_PIN); Wire.setClock(400000); Wire.setTimeOut(5);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x75);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom((int)MPU_ADDR, 1) != 1) return false;
  int who = Wire.read();
  Serial.printf("MPU WHO_AM_I = 0x%02X\n", who);
  bool ok = mpuWrite(0x6B, 0x01);   // wake, PLL on gyro X
  delay(50);
  ok &= mpuWrite(0x1A, 0x04);       // DLPF ~21 Hz
  ok &= mpuWrite(0x1B, 0x10);       // +-1000 dps -> 32.8 LSB/dps (wide range: a runaway spin must not clip the gyro)
  return ok;
}
bool readGyroZ(float &dps) {
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x47);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 2) != 2) return false;
  int hiB = Wire.read(); int loB = Wire.read();     // separate statements: evaluation order matters
  dps = (int16_t)((hiB << 8) | loB) / 32.8f;
  return true;
}
bool calibrateGyro() {              // robot MUST be stationary (~1.6 s per attempt)
  if (!imuOk) return false;
  state = ST_CAL; motorStop();
  bool ok = false;
  for (int attempt = 0; attempt < 3 && !ok; attempt++) {
    double sum = 0, sum2 = 0; int n = 0;
    for (int i = 0; i < 400; i++) {
      float r; if (readGyroZ(r)) { sum += r; sum2 += r * r; n++; }
      delay(4);
    }
    if (n < 300) { Serial.printf("gyro cal attempt %d: only %d valid reads\n", attempt, n); continue; }
    float mean = sum / n, sd = sqrtf(fmaxf(0.0f, (float)(sum2 / n - (double)mean * mean)));
    Serial.printf("gyro cal attempt %d: bias %.3f dps, sd %.3f dps\n", attempt, mean, sd);
    if (sd < 0.6f) { gyroBias = mean; ok = true; }   // else it moved: retry
  }
  gyroCal = ok; state = ST_IDLE;
  lastTickU = micros();
  return ok;
}

// ---------- Helpers ----------
float slew(float cur, float target, float maxStep) {
  float d = constrain(target - cur, -maxStep, maxStep); return cur + d;
}
void resetPids() { pidSpeedL.reset(); pidSpeedR.reset(); pidTurn.reset(); pidDist.reset(); pidHold.reset(); pidDir.reset(); }
void resyncSpeed() {                // after geometry / sign changes
  noInterrupts(); int32_t cl = encL, cr = encR; interrupts();
  const float mpc = (PI * wheelDiam) / countsPerRev;
  prevPosL = encSignL * cl * mpc; prevPosR = encSignR * cr * mpc * scaleR;
  for (int i = 0; i < SPD_WIN; i++) { hL[i] = prevPosL; hR[i] = prevPosR; }
  vL = vR = vLprev = vRprev = 0;
}
bool isrOn = false;
void applySides() {                 // map logical left/right onto the physical wheels (motor pair + encoder) according to swapSides
  motorStop();
  if (isrOn) { detachInterrupt(digitalPinToInterrupt(eLA)); detachInterrupt(digitalPinToInterrupt(eRA)); }
  if (!swapSides) { pL1 = WL_1; pL2 = WL_2; pR1 = WR_1; pR2 = WR_2; eLA = ENC_WL_A; eLB = ENC_WL_B; eRA = ENC_WR_A; eRB = ENC_WR_B; }
  else            { pL1 = WR_1; pL2 = WR_2; pR1 = WL_1; pR2 = WL_2; eLA = ENC_WR_A; eLB = ENC_WR_B; eRA = ENC_WL_A; eRB = ENC_WL_B; }
  noInterrupts(); encL = 0; encR = 0; interrupts();
  attachInterrupt(digitalPinToInterrupt(eLA), isrL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(eRA), isrR, CHANGE);
  isrOn = true;
  resyncSpeed();
}
void finishMove(bool timeout) {
  motorStop(); state = ST_IDLE; flagDone = !timeout; flagTimeout = timeout;
  resetPids(); wCmd = vCmd = 0; tolSince = 0;
}
void beginMove(uint8_t st) {
  resetPids(); wCmd = vCmd = 0; tolSince = 0; moveStart = millis(); badSince = stallSince = 0; openLoop = false; turnDirect = false;
  flagDone = flagTimeout = flagFault = false; repTgt = repErr = repAux = 0; state = st;
}
float activeHeading() { return useGyroNow ? thGyro : thEnc; }

// ---------- Telemetry / acks ----------
void sendAck(uint8_t kind, float a, float b, float c) {
  if (!haveTuner) return;
  Ack k = {0xD2, kind, a, b, c};
  udp.beginPacket(tunerIP, TELEM_PORT); udp.write((const uint8_t*)&k, sizeof(k)); udp.endPacket();
}
float limitFlags() {
  return (invL ? 1 : 0) | (invR ? 2 : 0) | (encSignL < 0 ? 4 : 0) | (encSignR < 0 ? 8 : 0) | (invGyro ? 16 : 0);
}
void sendAllAcks() {
  sendAck(0, pidSpeedL.kp, pidSpeedL.ki, pidSpeedL.kd);
  sendAck(1, pidTurn.kp, pidTurn.ki, pidTurn.kd);
  sendAck(2, pidDist.kp, pidDist.ki, pidDist.kd);
  sendAck(3, pidHold.kp, pidHold.ki, pidHold.kd);
  sendAck(4, pidDir.kp, pidDir.ki, pidDir.kd);
  sendAck(10, countsPerRev, wheelDiam, trackW);
  sendAck(11, minPwm, maxPwm, limitFlags());
  sendAck(12, headSrc, gyroScale, gyroCal ? gyroBias : 0);
  sendAck(13, scaleR, cfgSaved ? 1 : 0, 0);
}
void sendTelemetry() {
  if (!haveTuner) return;
  Telem t;
  t.magic = 0xD1; t.state = state; t.seq = seqNo++; t.t_ms = millis();
  t.f[0] = x; t.f[1] = y; t.f[2] = activeHeading(); t.f[3] = thGyro; t.f[4] = thEnc;
  t.f[5] = repTgt; t.f[6] = repErr; t.f[7] = repAux;
  t.f[8] = vL; t.f[9] = vR; t.f[10] = tgtL; t.f[11] = tgtR;
  t.pwmL = pwmL; t.pwmR = pwmR;
  t.flags = (imuOk ? 1 : 0) | (useGyroNow ? 2 : 0) | (flagTimeout ? 4 : 0) | (flagDone ? 8 : 0) | (gyroCal ? 16 : 0) | (flagFault ? 32 : 0);
  udp.beginPacket(tunerIP, TELEM_PORT); udp.write((const uint8_t*)&t, sizeof(t)); udp.endPacket();
}

// ---------- Sign test: run each wheel alone for 350 ms, report encoder and gyro signs ----------
void signTest() {
  if (state != ST_IDLE) return;
  state = ST_CAL; motorStop(); delay(300);
  long dl[2] = {0, 0}, dr[2] = {0, 0}; float gz[2] = {0, 0};
  for (int s = 0; s < 2; s++) {                       // s=0: left wheel only, s=1: right wheel only
    noInterrupts(); int32_t l0 = encL, r0 = encR; interrupts();
    float g = 0; uint32_t t0 = millis(), tp = micros();
    if (s == 0) driveSide(pL1, pL2, 0.25f, invL); else driveSide(pR1, pR2, 0.25f, invR);
    while (millis() - t0 < 350) {
      float dps; uint32_t tn = micros();
      if (imuOk && readGyroZ(dps)) g += (dps - gyroBias) * ((tn - tp) * 1e-6f);
      tp = tn; delay(2);
    }
    motorStop();
    noInterrupts(); int32_t l1 = encL, r1 = encR; interrupts();
    dl[s] = l1 - l0; dr[s] = r1 - r0; gz[s] = g;
    delay(500);
  }
  Serial.println("=== SIGN TEST (PWM fraction +0.25 on one wheel at a time, 350 ms) ===");
  Serial.printf("LEFT only : raw dL=%ld dR=%ld gyro=%.1f deg (before gyro flip/scale)\n", dl[0], dr[0], gz[0]);
  Serial.printf("RIGHT only: raw dL=%ld dR=%ld gyro=%.1f deg\n", dl[1], dr[1], gz[1]);
  bool lMoved = labs(dl[0]) > 5, rMoved = labs(dr[1]) > 5;
  Serial.printf("L encoder: %s\n", !lMoved ? "NO COUNTS (wheel stuck or encoder dead)" : (encSignL * dl[0] > 0 ? "sign OK" : "WRONG -> tick 'flip L encoder'"));
  Serial.printf("R encoder: %s\n", !rMoved ? "NO COUNTS (wheel stuck or encoder dead)" : (encSignR * dr[1] > 0 ? "sign OK" : "WRONG -> tick 'flip R encoder'"));
  float gs = invGyro ? -1.0f : 1.0f;
  bool gOk = gs * gz[0] < 0 && gs * gz[1] > 0, gInv = gs * gz[0] > 0 && gs * gz[1] < 0;
  Serial.printf("Gyro: %s\n", gOk ? "sign OK" : (gInv ? "WRONG -> tick 'flip gyro sign'" : "INCONSISTENT (a wheel did not turn the robot, or gyro not read)"));
  Serial.println("VISUAL CHECK: LEFT-only must curve the robot to its RIGHT, RIGHT-only to its LEFT.");
  Serial.println("If forward is right but LEFT-only curves the robot LEFT, press 'Swap left/right'.");
  Serial.println("If both are reversed (or the robot drove backwards), tick 'invert L motor' and 'invert R motor', then re-run.");
  state = ST_IDLE; lastTickU = micros();
}

// ---------- Auto-calibration (blocking, robot on the floor; STOP in the tuner aborts) ----------
bool calAborted() {
  int n;
  while ((n = udp.parsePacket()) > 0) {
    if (n == (int)sizeof(Cmd)) {
      Cmd c; udp.read((uint8_t*)&c, sizeof(c));
      if (c.magic == 0xC2 && c.type == C_STOP) calAbort = true;
    } else { while (udp.available()) udp.read(); }
  }
  return calAbort;
}
bool calWait(uint32_t ms) {          // waits, integrates the raw gyro, streams telemetry, honours STOP
  uint32_t t0 = millis(), lastTx = t0, tp = micros();
  while (millis() - t0 < ms) {
    uint32_t tn = micros();
    if (imuOk && gyroCal) { float dps; if (readGyroZ(dps)) calGyroDeg += (dps - gyroBias) * ((tn - tp) * 1e-6f); }
    tp = tn;
    if (calAborted()) { motorStop(); return false; }
    if (millis() - lastTx >= 40) { lastTx = millis(); sendTelemetry(); }
    delay(2);
  }
  return true;
}
bool calFinish(uint8_t step, int code, float aux) {   // code: 1 OK, 2 FAILED, 3 aborted
  motorStop(); pwmL = pwmR = 0;
  resyncSpeed();
  x = y = 0; thGyro = thEnc = thPrev = 0; pathS = 0;
  resetPids(); wCmd = vCmd = 0;
  state = ST_IDLE; lastTickU = micros();
  Serial.printf("AUTO-CAL step %d: %s\n", step, code == 1 ? "OK" : (code == 3 ? "ABORTED" : "FAILED"));
  sendAllAcks();
  sendAck(23, step, code, aux);      // last, so the tuner sees fresh values when it sees the verdict
  return code == 1;
}

// Step 1: which encoder sign makes 'positive command' read positive, is the gyro sign consistent, do both wheels push the same way.
bool calPolarity() {
  calAbort = false; state = ST_CAL; motorStop(); sendAck(23, 1, 0, 0);
  Serial.println("=== AUTO-CAL 1: polarity (robot on the floor, 30 cm free all around) ===");
  if (!calWait(400)) return calFinish(1, 3, 0);
  const float mpc = (PI * wheelDiam) / countsPerRev;
  int P = constrain((minPwm + maxPwm) / 2, 80, 160);
  bool gyroOk = imuOk && gyroCal;
  int32_t l0, r0, l1, r1;
  for (int pass = 0; pass < 2; pass++) {
    int32_t own[2] = {0, 0}; bool coupled = false;
    for (int s = 0; s < 2; s++) {                       // one wheel at a time
      int32_t cnt = 0, oth = 0;
      for (int attempt = 0; attempt < 2; attempt++) {
        int pw = (attempt == 0) ? P : constrain(P + 50, 100, 200);
        readEnc(l0, r0);
        rawDrive(s == 0 ? pw : 0, s == 1 ? pw : 0);
        if (!calWait(400)) return calFinish(1, 3, 0);
        motorStop();
        if (!calWait(300)) return calFinish(1, 3, 0);
        readEnc(l1, r1);
        cnt = (s == 0) ? (l1 - l0) : (r1 - r0);
        oth = (s == 0) ? (r1 - r0) : (l1 - l0);
        if (labs(cnt) >= 8) break;
      }
      Serial.printf("%s wheel alone: own encoder %ld counts, other encoder %ld counts\n", s == 0 ? "LEFT" : "RIGHT", (long)cnt, (long)oth);
      if (labs(cnt) < 8) {
        Serial.println("FAIL: that wheel's encoder did not count (motor stuck, dead encoder, or encoder pins/wiring wrong)");
        return calFinish(1, 2, 10 + s);
      }
      if (labs(oth) * 2 > labs(cnt)) coupled = true;
      own[s] = cnt;
    }
    if (coupled) {
      Serial.println("FAIL: counts appeared on the OTHER wheel's encoder: left/right encoder wires (or pins) are swapped");
      return calFinish(1, 2, 20);
    }
    encSignL = own[0] > 0 ? 1 : -1; encSignR = own[1] > 0 ? 1 : -1;   // positive command now always reads positive
    Serial.printf("encoder signs set: L %+d, R %+d (positive command -> positive count)\n", encSignL, encSignR);

    readEnc(l0, r0); calGyroDeg = 0;                    // straight nudge, both wheels positive
    rawDrive(P, P);
    if (!calWait(400)) return calFinish(1, 3, 0);
    motorStop(); if (!calWait(500)) return calFinish(1, 3, 0);
    readEnc(l1, r1);
    float sL = encSignL * (l1 - l0) * mpc, sR = encSignR * (r1 - r0) * mpc * scaleR;
    Serial.printf("STRAIGHT nudge (both wheels +): L %.0f mm, R %.0f mm by encoders (scale not verified yet), gyro heading change %.1f deg\n", sL * 1000, sR * 1000, calGyroDeg);
    if (gyroOk && fabsf(calGyroDeg) > 20.0f && pass == 0) {
      // Both wheels + should translate. A big rotation means one motor pushes against the other. The encoders cannot see this, the gyro can.
      invR = !invR;
      Serial.println("Both wheels + made the robot ROTATE: one motor is wired against the other. Inverting the RIGHT motor and repeating.");
      if (!calWait(500)) return calFinish(1, 3, 0);
      continue;
    }
    if (!gyroOk) Serial.println("NOTE: no calibrated gyro, so the robot cannot judge this itself. If it ROTATED instead of moving straight, press 'Swap right motor' and it will re-run.");
    Serial.println(">>> WATCH THE ROBOT: both wheels + must move it STRAIGHT toward its FRONT. Rotated -> 'Swap right motor'. Straight but toward the BACK -> 'Flip front/back'.");
    break;
  }

  readEnc(l0, r0); calGyroDeg = 0;                      // spin: left negative, right positive (= CCW in this firmware)
  rawDrive(-P, P);
  if (!calWait(600)) return calFinish(1, 3, 0);
  motorStop(); if (!calWait(500)) return calFinish(1, 3, 0);
  readEnc(l1, r1);
  float dL = encSignL * (l1 - l0) * mpc, dR = encSignR * (r1 - r0) * mpc * scaleR;
  float encDeg = (dR - dL) / trackW / D2R, gyroRaw = calGyroDeg;
  Serial.printf("SPIN (L negative, R positive): wheels L %.0f mm R %.0f mm, encoder heading %.1f deg, raw gyro %.1f deg\n", dL * 1000, dR * 1000, encDeg, gyroRaw);
  bool gyroChecked = false;
  if (imuOk && gyroCal && fabsf(encDeg) > 10.0f && fabsf(gyroRaw) > 10.0f) { invGyro = (gyroRaw * encDeg < 0); gyroChecked = true; }
  Serial.printf("Gyro sign: %s\n", gyroChecked ? (invGyro ? "flipped to agree with the wheels" : "already agrees with the wheels") : "NOT checked (no calibrated gyro, or the spin was too small)");
  sendAck(24, limitFlags(), gyroRaw, encDeg);
  saveConfig();
  return calFinish(1, 1, gyroChecked ? 1 : 0);
}

bool calSpeed(int pl, int pr, float &vl, float &vr) {   // steady wheel speeds (m/s, magnitude) at a raw PWM
  const float mpc = (PI * wheelDiam) / countsPerRev;
  int32_t l0, r0, l1, r1;
  rawDrive(pl, pr);
  if (!calWait(450)) return false;
  readEnc(l0, r0); uint32_t ta = millis();
  if (!calWait(300)) return false;
  readEnc(l1, r1); float dt = (millis() - ta) * 1e-3f;
  motorStop();
  vl = fabsf((l1 - l0) * mpc / dt); vr = fabsf((r1 - r0) * mpc * scaleR / dt);
  return calWait(400);
}

// Step 2: breakaway PWM (straight and spin) -> minPwm, then wheel speed at min and max PWM.
bool calDeadband() {
  calAbort = false; state = ST_CAL; motorStop(); sendAck(23, 2, 0, 0);
  Serial.println("=== AUTO-CAL 2: deadband and speed range (run step 1 first; robot on the floor, 60 cm free) ===");
  if (!calWait(400)) return calFinish(2, 3, 0);
  int brk[2][2] = {{0, 0}, {0, 0}};                     // [0 straight, 1 spin][left, right]
  for (int ph = 0; ph < 2; ph++) {
    int dirL = (ph == 0) ? 1 : -1;
    int32_t l0, r0, l1, r1; readEnc(l0, r0);
    for (int pw = 30; pw <= 190; pw += 2) {
      rawDrive(dirL * pw, pw);
      if (!calWait(90)) return calFinish(2, 3, 0);
      readEnc(l1, r1);
      int dl = dirL * encSignL * (int)(l1 - l0), dr = encSignR * (int)(r1 - r0);
      l0 = l1; r0 = r1;
      if (!brk[ph][0] && dl >= 5) brk[ph][0] = pw;
      if (!brk[ph][1] && dr >= 5) brk[ph][1] = pw;
      int fb = brk[ph][0];
      if (brk[ph][1] && (!fb || brk[ph][1] < fb)) fb = brk[ph][1];
      if (brk[ph][0] && brk[ph][1]) break;
      if (fb && pw >= fb + 30) break;                   // one wheel moved, the other never did: stop ramping
    }
    motorStop();
    Serial.printf("%s: wheels start moving at PWM L=%d R=%d (0 = never moved)\n", ph == 0 ? "straight" : "spin", brk[ph][0], brk[ph][1]);
    if (!brk[ph][0] || !brk[ph][1]) {
      Serial.println("FAIL: a wheel never moved (run step 1, check battery, motor, encoder)");
      return calFinish(2, 2, 10 + ph);
    }
    if (!calWait(600)) return calFinish(2, 3, 0);
  }
  int worst = brk[0][0];
  for (int a = 0; a < 2; a++) for (int b = 0; b < 2; b++) if (brk[a][b] > worst) worst = brk[a][b];
  int newMin = constrain(worst + 2, 20, 200);
  int newMax = maxPwm;
  if (newMax < newMin + 30) newMax = constrain(newMin + 60, 0, 255);
  minPwm = newMin; maxPwm = newMax;
  float vl = 0, vr = 0, vMinS = 0, vMaxS = 0, vSpin = 0;
  if (!calSpeed(minPwm, minPwm, vl, vr)) return calFinish(2, 3, 0);
  vMinS = 0.5f * (vl + vr);
  if (!calSpeed(maxPwm, maxPwm, vl, vr)) return calFinish(2, 3, 0);
  vMaxS = 0.5f * (vl + vr);
  if (!calSpeed(-maxPwm, maxPwm, vl, vr)) return calFinish(2, 3, 0);
  vSpin = 0.5f * (vl + vr);
  float spinDps = vSpin * 2.0f / trackW / D2R;
  Serial.printf("minPwm = %d (worst breakaway %d + 2), maxPwm = %d\n", minPwm, worst, maxPwm);
  Serial.printf("wheel speed at min PWM %.3f m/s, at max PWM %.3f m/s; spin at max PWM %.3f m/s = %.0f deg/s (distance scale not verified yet)\n", vMinS, vMaxS, vSpin, spinDps);
  Serial.printf("suggested limits: drive max <= %.2f m/s, turn max <= %.0f deg/s, lowest controllable wheel speed %.3f m/s\n", 0.7f * vMaxS, 0.6f * spinDps, vMinS);
  sendAck(20, brk[0][0], brk[0][1], minPwm);
  sendAck(21, brk[1][0], brk[1][1], maxPwm);
  sendAck(22, vMinS, vMaxS, vSpin);
  saveConfig();
  return calFinish(2, 1, 0);
}

// Step 3: counts per wheel revolution from a hand rotation (the only reference the robot cannot supply itself).
void calHand(int phase, float revs) {
  if (phase == 1) {
    motorStop(); state = ST_CAL; handCap = true; readEnc(capL0, capR0);
    sendAck(23, 3, 0, 0);
    Serial.println("=== AUTO-CAL 3: lift the robot, mark each tyre, turn EACH wheel exactly N full turns by hand, then press Finish with N ===");
    return;
  }
  if (!handCap) return;
  int32_t l, r; readEnc(l, r);
  float dL = fabsf((float)(l - capL0)), dR = fabsf((float)(r - capR0));
  handCap = false;
  float cprL = (revs >= 0.5f) ? dL / revs : 0.0f, cprR = (revs >= 0.5f) ? dR / revs : 0.0f;
  float sr = (cprR > 0.0f) ? cprL / cprR : 0.0f;
  Serial.printf("hand rotation: L %.0f counts, R %.0f counts over %.2f turns -> counts/rev L %.1f, R %.1f\n", dL, dR, revs, cprL, cprR);
  if (dL < 100 || dR < 100 || cprL < 50 || cprL > 20000 || sr < 0.7f || sr > 1.43f) {
    Serial.println("FAIL: not applied (a wheel did not count, N missing, or the two wheels differ by more than 40%)");
    calFinish(3, 2, 0);
    return;
  }
  countsPerRev = cprL; scaleR = sr;                       // left is the reference, right is scaled to match
  saveConfig();
  sendAck(26, cprL, cprR, scaleR);
  Serial.printf("applied: counts/rev %.1f, right scale %.4f (diameter %.1f mm was NOT measured by this step)\n", countsPerRev, scaleR, wheelDiam * 1000);
  calFinish(3, 1, 0);
}

// Step 4: effective track width from a gyro-referenced 360 degree spin each way (needs step 1, a calibrated gyro and a good distance scale).
bool calTrack() {
  calAbort = false; state = ST_CAL; motorStop(); sendAck(23, 4, 0, 0);
  Serial.println("=== AUTO-CAL 4: track width from the gyro (robot on the floor, 30 cm free) ===");
  if (!(imuOk && gyroCal)) { Serial.println("FAIL: needs a working, calibrated gyro"); return calFinish(4, 2, 1); }
  const float mpc = (PI * wheelDiam) / countsPerRev;
  int P = constrain(minPwm + 20, 60, 200);
  float ratio[2] = {0, 0};
  if (!calWait(400)) return calFinish(4, 3, 0);
  for (int k = 0; k < 2; k++) {
    int dir = (k == 0) ? 1 : -1;
    int32_t l0, r0, l1, r1; readEnc(l0, r0); calGyroDeg = 0;
    rawDrive(-dir * P, dir * P);
    uint32_t t0 = millis(); bool reached = false;
    while (millis() - t0 < 9000) {
      if (!calWait(10)) return calFinish(4, 3, 0);
      if (fabsf(calGyroDeg * gyroScale) >= 340.0f) { reached = true; break; }
    }
    motorStop();
    if (!reached) { Serial.println("FAIL: the spin did not reach 340 deg in 9 s (stalled? run step 2)"); return calFinish(4, 2, 2); }
    if (!calWait(700)) return calFinish(4, 3, 0);       // coast to rest while the gyro keeps integrating
    readEnc(l1, r1);
    float dL = encSignL * (l1 - l0) * mpc, dR = encSignR * (r1 - r0) * mpc * scaleR;
    float encDeg = fabsf((dR - dL) / trackW / D2R), gyDeg = fabsf(calGyroDeg * gyroScale);
    ratio[k] = (gyDeg > 1.0f) ? encDeg / gyDeg : 0.0f;
    Serial.printf("%s spin: gyro %.1f deg, encoders (with current track) %.1f deg, ratio %.4f\n", k == 0 ? "CCW" : "CW", gyDeg, encDeg, ratio[k]);
    if (!calWait(600)) return calFinish(4, 3, 0);
  }
  float r = 0.5f * (ratio[0] + ratio[1]);
  if (!(r > 0.5f && r < 2.0f)) { Serial.println("FAIL: ratio implausible (check step 1 and the distance scale)"); return calFinish(4, 2, 3); }
  if (fabsf(ratio[0] - ratio[1]) > 0.06f * r) Serial.println("WARNING: CCW and CW ratios differ by more than 6%: slip or asymmetry, treat the result as approximate");
  trackW *= r;
  Serial.printf("effective track width set to %.1f mm (gyro scale assumed %.3f)\n", trackW * 1000, gyroScale);
  sendAck(27, trackW, ratio[0], ratio[1]);
  saveConfig();
  return calFinish(4, 1, 0);
}

// ---------- Commands ----------
void handleUdp() {
  int n = udp.parsePacket();
  if (n <= 0) return;
  IPAddress from = udp.remoteIP();
  if (n != (int)sizeof(Cmd)) { while (udp.available()) udp.read(); return; }
  Cmd c; udp.read((uint8_t*)&c, sizeof(c));
  if (c.magic != 0xC2) return;
  tunerIP = from; haveTuner = true;
  switch (c.type) {
    case C_STOP:  motorStop(); handCap = false; state = ST_IDLE; flagDone = flagTimeout = flagFault = false; resetPids(); wCmd = vCmd = 0; break;
    case C_TURN:
      if (state != ST_IDLE) break;
      turnStart = activeHeading(); turnTarget = turnStart + c.a * D2R;
      wMax = constrain(c.b, 5.0f, 180.0f) * D2R;
      turnPath0 = pathS; beginMove(ST_TURN); repTgt = turnTarget - turnStart; break;
    case C_DRIVE:
      if (state != ST_IDLE) break;
      distTarget = c.a; vMax = constrain(c.b, 0.02f, 0.5f);
      pathStart = pathS; holdTarget = activeHeading();
      beginMove(ST_DRIVE); repTgt = distTarget; break;
    case C_WHEEL:
      if (state != ST_IDLE) break;
      cmdVL = constrain(c.a, -0.5f, 0.5f); cmdVR = constrain(c.b, -0.5f, 0.5f);
      wheelEnd = millis() + (uint32_t)(constrain(c.c <= 0 ? 2.0f : c.c, 0.2f, 5.0f) * 1000);
      beginMove(ST_WHEEL); openLoop = c.d > 0.5f; break;   // d=1: values are PWM fractions, no PID
    case C_GAINS: {
      PID* p[2] = {nullptr, nullptr}; int id = (int)c.a;
      if (id == 0) { p[0] = &pidSpeedL; p[1] = &pidSpeedR; }
      else if (id == 1) p[0] = &pidTurn; else if (id == 2) p[0] = &pidDist; else if (id == 3) p[0] = &pidHold; else if (id == 4) p[0] = &pidDir;
      for (int i = 0; i < 2; i++) if (p[i]) { p[i]->kp = c.b; p[i]->ki = c.c; p[i]->kd = c.d; p[i]->reset(); }
      if (p[0]) sendAck(id, c.b, c.c, c.d);
      break; }
    case C_GEOM:
      if (c.a > 10 && c.b > 0.005f && c.c > 0.02f) { countsPerRev = c.a; wheelDiam = c.b; trackW = c.c; if (c.d > 0.5f && c.d < 2.0f) scaleR = c.d; resyncSpeed(); }
      sendAck(10, countsPerRev, wheelDiam, trackW); sendAck(13, scaleR, cfgSaved ? 1 : 0, 0); break;
    case C_LIMITS: {
      int mn = (int)c.a, mx = (int)c.b, f = (int)c.c;
      if (mn >= 0 && mx > mn && mx <= 255) { minPwm = mn; maxPwm = mx; }
      invL = f & 1; invR = f & 2; encSignL = (f & 4) ? -1 : 1; encSignR = (f & 8) ? -1 : 1; invGyro = f & 16;
      resyncSpeed(); sendAck(11, minPwm, maxPwm, limitFlags()); break; }
    case C_ZERO:
      if (state == ST_IDLE) { x = y = 0; thGyro = thEnc = thPrev = 0; pathS = 0; }
      break;
    case C_HEAD:
      headSrc = (c.a > 0.5f) ? 1 : 0; if (c.b > 0.5f && c.b < 1.5f) gyroScale = c.b;
      sendAck(12, headSrc, gyroScale, gyroCal ? gyroBias : 0); break;
    case C_CALIB:
      if (state == ST_IDLE) { if (!imuOk) { imuOk = mpuInit(); Serial.println(imuOk ? "IMU re-init OK" : "IMU re-init FAILED"); } imuFail = 0; calibrateGyro(); sendAck(12, headSrc, gyroScale, gyroCal ? gyroBias : 0); }
      break;
    case C_REQ: sendAllAcks(); break;
    case C_SIGN: signTest(); break;
    case C_TURNG:                       // gyro-only turn: a = delta deg, b = max PWM fraction, c = stop lead time (s)
      if (state != ST_IDLE) break;
      if (!useGyroNow) { Serial.println("gyro-only turn needs a working, calibrated gyro"); break; }
      turnStart = thGyro; turnTarget = turnStart + c.a * D2R;
      dirMax = constrain(c.b, 0.05f, 0.6f); turnLead = constrain(c.c <= 0 ? 0.05f : c.c, 0.0f, 0.3f);
      turnPath0 = pathS; beginMove(ST_TURN); turnDirect = true; repTgt = turnTarget - turnStart; break;
    case C_ACAL: {
      int step = (int)c.a;
      if (state != ST_IDLE && !(step == 3 && handCap)) break;
      if (step == 1) calPolarity();
      else if (step == 2) calDeadband();
      else if (step == 3) calHand((int)c.b, c.c);
      else if (step == 4) calTrack();
      break; }
    case C_SAVE:
      saveConfig(); sendAck(13, scaleR, cfgSaved ? 1 : 0, 0);
      Serial.println(cfgSaved ? "settings saved to flash" : "SAVE FAILED"); break;
    case C_FLIPFB:                      // the one fact the robot cannot sense: which end is the front
      if (state != ST_IDLE) break;
      { bool a = invL, b = invR; invL = !b; invR = !a; }   // other end is the front: sides trade places AND both motors run the other way
      swapSides = !swapSides; applySides();
      Serial.println("front/back flipped: left/right trade places and both motors are inverted; re-deriving signs");
      calPolarity(); break;
    case C_SWAPLR:                      // forward is right, but the wheel the firmware calls 'left' is on the robot's right
      if (state != ST_IDLE) break;
      { bool a = invL; invL = invR; invR = a; }
      swapSides = !swapSides; applySides();
      Serial.println("left/right swapped (motors, encoders and invert flags move together); re-deriving signs");
      calPolarity(); break;
    case C_FLIPR:                       // fix 'drive rotates / turn translates': one motor is wired against the other
      if (state != ST_IDLE) break;
      invR = !invR;
      Serial.println("right motor direction inverted; re-deriving signs");
      calPolarity(); break;
    case C_RSTCFG:
      if (prefs.begin("rtune", false)) { prefs.clear(); prefs.end(); }
      cfgSaved = false; sendAck(13, scaleR, 0, 0);
      Serial.println("saved settings erased (the current values stay until the next reboot)"); break;
    default: break;   // C_HELLO: only registers the tuner
  }
}

// ---------- Control tick (100 Hz) ----------
void controlTick(float dt) {
  uint32_t now = millis();

  // encoders -> wheel positions, speeds, encoder heading, path length
  noInterrupts(); int32_t cl = encL, cr = encR; interrupts();
  const float mpc = (PI * wheelDiam) / countsPerRev;
  float posL = encSignL * cl * mpc, posR = encSignR * cr * mpc * scaleR;
  float dSl = posL - prevPosL, dSr = posR - prevPosR;
  prevPosL = posL; prevPosR = posR;
  float ds = 0.5f * (dSl + dSr), dthEnc = (dSr - dSl) / trackW;
  thEnc += dthEnc; pathS += ds;
  hi = (hi + 1) % SPD_WIN;
  float vlRaw = (posL - hL[hi]) / (SPD_WIN * dt), vrRaw = (posR - hR[hi]) / (SPD_WIN * dt);
  hL[hi] = posL; hR[hi] = posR;
  vLprev = vL; vRprev = vR;
  vL += SPEED_LPF * (vlRaw - vL); vR += SPEED_LPF * (vrRaw - vR);
  float aL = (vL - vLprev) / dt, aR = (vR - vRprev) / dt;
  rateEncF += 0.2f * (dthEnc / dt - rateEncF);

  // gyro
  if (imuOk) {
    float dps;
    if (readGyroZ(dps)) { imuFail = 0; wGyro = (dps - gyroBias) * (invGyro ? -1.0f : 1.0f) * gyroScale * D2R; }
    else if (++imuFail > 20) { imuOk = false; wGyro = 0; Serial.println("IMU dropped: 20 consecutive gyro read failures"); }
    thGyro += wGyro * dt;
  }
  useGyroNow = imuOk && gyroCal && headSrc == 1;
  float th = activeHeading(), rate = useGyroNow ? wGyro : rateEncF;
  float dth = th - thPrev; thPrev = th;
  x += ds * cosf(th - 0.5f * dth); y += ds * sinf(th - 0.5f * dth);

  // motion state machine -> wheel speed targets
  float vlT = 0, vrT = 0, olL = 0, olR = 0; bool run = false, direct = false, brake = false;
  if (state == ST_WHEEL) {
    run = true; vlT = openLoop ? 0.0f : cmdVL; vrT = openLoop ? 0.0f : cmdVR;
    if (now >= wheelEnd) { finishMove(false); run = false; vlT = vrT = 0; }
  } else if (state == ST_TURN) {
    run = true;
    float e = turnTarget - th;
    bool inTol = fabsf(e) < TURN_TOL && fabsf(rate) < TURN_RATE_TOL;
    repErr = e; repAux = 0;
    if (turnDirect) {                 // gyro-only: heading PD -> differential PWM, brake when predicted to reach the target
      direct = true;
      float sgn = (turnTarget - turnStart) >= 0 ? 1.0f : -1.0f;
      float prog = (th - turnStart) * sgn, want = fabsf(turnTarget - turnStart);
      float ePred = e - rate * turnLead;            // where the error will be after 'turnLead' seconds
      float u = pidDir.update(e, rate, dt, dirMax); // + = CCW
      if (prog < -20.0f * D2R || prog > want + 45.0f * D2R) {
        Serial.println("FAULT: gyro turn goes the wrong way or runs away (check invert motor / flip gyro sign)");
        finishMove(true); flagFault = true; run = false;
      } else if (inTol || e * ePred <= 0) { brake = true; }
      else { olL = -u; olR = u; }
    } else {
      float wDes = pidTurn.update(e, rate, dt, wMax);
      wCmd = slew(wCmd, wDes, W_SLEW * dt);
      vlT = -wCmd * trackW * 0.5f; vrT = wCmd * trackW * 0.5f;
    }
    if (run) {
      if (inTol) { if (!tolSince) tolSince = now; if (now - tolSince >= SETTLE_MS) { finishMove(false); run = false; } }
      else tolSince = 0;
      if (state == ST_TURN && now - moveStart > MOVE_TIMEOUT_MS) { finishMove(true); run = false; }
    }
    if (run && fabsf(pathS - turnPath0) > 0.05f) {   // a pure spin keeps the mean wheel travel near zero
      Serial.println("FAULT: the robot is translating during a turn (wheels moving the same way): run Auto-cal 1 and check the motor polarity");
      finishMove(true); flagFault = true; run = false;
    }
  } else if (state == ST_DRIVE) {
    run = true;
    float s = pathS - pathStart, e = distTarget - s, vMeas = 0.5f * (vL + vR);
    float vDes = pidDist.update(e, vMeas, dt, vMax);
    vCmd = slew(vCmd, vDes, V_SLEW * dt);
    float eh = holdTarget - th;
    float wCorr = pidHold.update(eh, rate, dt, W_HOLD_MAX);
    vlT = vCmd - wCorr * trackW * 0.5f; vrT = vCmd + wCorr * trackW * 0.5f;
    repErr = e; repAux = eh;
    bool inTol = fabsf(e) < DRIVE_TOL && fabsf(vMeas) < DRIVE_V_TOL;
    if (inTol) { if (!tolSince) tolSince = now; if (now - tolSince >= SETTLE_MS) { finishMove(false); run = false; } }
    else tolSince = 0;
    if (state == ST_DRIVE && now - moveStart > MOVE_TIMEOUT_MS) { finishMove(true); run = false; }
  }
  if (!run) { vlT = vrT = 0; }
  tgtL = vlT; tgtR = vrT;

  // inner wheel-speed loops
  if (run) {
    float uL = 0, uR = 0;
    if (state == ST_WHEEL && openLoop) { uL = cmdVL; uR = cmdVR; }          // bring-up: raw PWM fraction, no feedback
    else if (direct) { uL = olL; uR = olR; }                                 // gyro-only turn: no encoder feedback
    else {
      if (fabsf(vlT) < V_MIN_CMD) pidSpeedL.reset(); else uL = pidSpeedL.update(vlT - vL, aL, dt, 1.0f);
      if (fabsf(vrT) < V_MIN_CMD) pidSpeedR.reset(); else uR = pidSpeedR.update(vrT - vR, aR, dt, 1.0f);
      // one-sided loop: never apply reverse torque to a wheel whose target points the other way. At low speed the +/- minPwm
      // reversal every few ticks WAS the chatter. The wheel now coasts instead of being driven backwards.
      if (vlT > 0)      { if (uL < 0) uL = 0; if (pidSpeedL.iTerm < 0) pidSpeedL.iTerm = 0; }
      else if (vlT < 0) { if (uL > 0) uL = 0; if (pidSpeedL.iTerm > 0) pidSpeedL.iTerm = 0; }
      if (vrT > 0)      { if (uR < 0) uR = 0; if (pidSpeedR.iTerm < 0) pidSpeedR.iTerm = 0; }
      else if (vrT < 0) { if (uR > 0) uR = 0; if (pidSpeedR.iTerm > 0) pidSpeedR.iTerm = 0; }
      // fault guards: a wrong encoder sign makes the loop positive feedback (full PWM forever)
      bool wrong = (fabsf(vlT) > V_MIN_CMD && vlT * vL < 0 && fabsf(vL) > 0.05f) ||
                   (fabsf(vrT) > V_MIN_CMD && vrT * vR < 0 && fabsf(vR) > 0.05f);
      bool stalled = (fabsf(vlT) > 0.03f && fabsf(vL) < 0.003f) || (fabsf(vrT) > 0.03f && fabsf(vR) < 0.003f);
      if (wrong) { if (!badSince) badSince = now; } else badSince = 0;
      if (stalled) { if (!stallSince) stallSince = now; } else stallSince = 0;
      if (badSince && now - badSince > 300) {
        Serial.println("FAULT: wheel measured moving opposite to command: run Auto-cal 1 (polarity)");
        finishMove(true); flagFault = true; run = false;
      } else if (stallSince && now - stallSince > 1500) {
        Serial.println("FAULT: no wheel motion measured (dead encoder or stalled motor)");
        finishMove(true); flagFault = true; run = false;
      }
    }
    if (run) {
      if (brake) { motorBrake(); pwmL = pwmR = 0; }
      else { pwmL = driveSide(pL1, pL2, uL, invL); pwmR = driveSide(pR1, pR2, uR, invR); }
    }
  }
  if (!run) { motorStop(); pwmL = pwmR = 0; pidSpeedL.reset(); pidSpeedR.reset(); }
}

void setup() {
  motorStop();                       // first thing: nothing floats
  Serial.begin(115200);
  Serial.println(loadConfig() ? "settings loaded from flash" : "no saved settings: using defaults");
  pinMode(ENC_WL_A, INPUT); pinMode(ENC_WL_B, INPUT);
  pinMode(ENC_WR_A, INPUT); pinMode(ENC_WR_B, INPUT);
  applySides();                                                      // attaches the two encoder interrupts for the current left/right assignment
  attachInterrupt(digitalPinToInterrupt(ENC_WL_B), isrLB, CHANGE);   // diagnostic only
  Serial.println(swapSides ? "sides: logical LEFT = M1A wheel (U5, enc 19/18), RIGHT = M2A wheel (U2, enc 34/25)"
                           : "sides: logical LEFT = M2A wheel (U2, enc 34/25), RIGHT = M1A wheel (U5, enc 19/18)");

  imuOk = mpuInit();
  Serial.printf("MPU6050: %s\n", imuOk ? "found" : "NOT found (heading falls back to encoders)");
  if (imuOk) {
    Serial.println("Keep the robot still: calibrating gyro...");
    Serial.println(calibrateGyro() ? "gyro calibrated" : "gyro calibration FAILED (robot moved?)");
  }

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(250); Serial.print("."); }
  WiFi.setSleep(false);
  Serial.printf("\nWiFi OK, robot IP %s, command port %u\n", WiFi.localIP().toString().c_str(), CMD_PORT);
  udp.begin(CMD_PORT);
  lastTickU = micros();
}

void loop() {
  handleUdp();
  uint32_t nowU = micros();
  if (nowU - lastTickU >= CTRL_US) {
    float dt = (nowU - lastTickU) * 1e-6f;
    lastTickU = nowU;
    if (dt > 0.03f) dt = 0.03f;
    controlTick(dt);
    if ((++tickCount & 1) == 0) sendTelemetry();   // 50 Hz
  }
  static uint32_t lastCap = 0;                     // Auto-cal 3: live hand-rotation counts to the tuner
  if (handCap && millis() - lastCap >= 200) { lastCap = millis(); int32_t l, r; readEnc(l, r); sendAck(25, (float)(l - capL0), (float)(r - capR0), 0); }
  // encoder diagnostic: raw counts to the serial monitor (2 Hz, only when something changed)
  static uint32_t lastPrint = 0, prE = 0; static int32_t prL = 0, prR = 0;
  if (millis() - lastPrint >= 500) {
    lastPrint = millis();
    noInterrupts(); int32_t cl = encL, cr = encR; interrupts();
    uint32_t e = edgesLB;
    if (cl != prL || cr != prR || e != prE) {
      Serial.printf("ENC raw L=%ld R=%ld (change dL=%ld dR=%ld) | LB edges=%lu, levels LA=%d LB=%d\n", (long)cl, (long)cr, (long)(cl - prL), (long)(cr - prR), (unsigned long)e, digitalRead(eLA), digitalRead(eLB));
      prL = cl; prR = cr; prE = e;
    }
  }
}