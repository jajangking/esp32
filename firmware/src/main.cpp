#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <config.h>

// Sensors
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <MPU6050_6Axis_MotionApps20.h>

// Servo
#include <ESP32Servo.h>

// ── Objects ──────────────────────────────────────────────────────────────────
WebServer server(80);
Adafruit_VL53L0X tof;
MPU6050 mpu;
bool dmpReady = false;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Servo servo;
Preferences prefs;

// ── Motor state ──────────────────────────────────────────────────────────────
struct {
  int16_t targetL  = 0;
  int16_t targetR  = 0;
  int16_t currentL = 0;
  int16_t currentR = 0;
  unsigned long lastCmdTime = 0;
} motor;

// ── Application state ────────────────────────────────────────────────────────
struct {
  // Motor
  int16_t leftMotor  = 0;
  int16_t rightMotor = 0;
  int16_t speedLimit = DEFAULT_SPEED_LIMIT;
  int16_t trimL      = 0;
  int16_t trimR      = 0;

  // Behavior
  String  behavior   = "stop";
  bool    emergency  = false;

  // Servo
  uint8_t servoAng   = 90;

  // Sensor readings
  uint16_t distance  = 0;
  uint16_t lastGoodDistance = 0;
  uint32_t  vlTimingBudget = VL_TIMING_BUDGET_DEFAULT;
  int16_t   vlOffset = 0;
  String    vlPreset = "standard";
  float     vlSignalRate = 0;
  float     vlAmbientRate = 0;
  bool      vlValid = false;
  bool      vlAuto = true;
  int16_t  sectors[3] = {-1, -1, -1};
  float    yaw   = 0;
  float    roll  = 0;
  float    pitch = 0;
  float    gyroX = 0;
  float    gyroY = 0;
  float    gyroZ = 0;
  float    gyroOffsetX = 0;
  float    gyroOffsetY = 0;
  float    gyroOffset = 0;
  bool     tiltDetected = false;
  bool     collisionDetected = false;
  unsigned long collisionTime = 0;
  float    accelMagnitude = 0;
  float    temperature = 0;

  // Tuning
  uint16_t safeDist   = DEFAULT_SAFE_DIST_MM;
  uint8_t  exploreSpd = DEFAULT_EXPLORE_SPEED;

  // Telemetry
  unsigned long uptime    = 0;
  int32_t       rssi      = 0;
  uint32_t      heap      = 0;
  String        ssid;
  bool          sensorOk  = false;
  bool          mpuOk     = false;

  // Log ring
  String logBuf[LOG_POLL_MAX];
  int    logIdx = 0;
  int    logCnt = 0;

  // Firmware info
  const char* version = "2.0.0";
} state;

unsigned long lastTelemetry = 0;
unsigned long lastScanMs    = 0;
unsigned long lastMotorMs   = 0;
unsigned long lastLEDMs     = 0;
// ── Scan state machine ───────────────────────────────────────────────────────
enum ScanStep { SCAN_IDLE, SCAN_L, SCAN_F, SCAN_R };
ScanStep scanStep = SCAN_IDLE;

// ── Explore state machine ────────────────────────────────────────────────────
enum ExplorePhase { EXP_FWD, EXP_REV, EXP_TURN };
ExplorePhase explorePhase = EXP_FWD;
unsigned long phaseStart   = 0;

// ── Helpers ──────────────────────────────────────────────────────────────────
void logMsg(const char *fmt, ...) {
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  state.logBuf[state.logIdx] = String(buf);
  state.logIdx = (state.logIdx + 1) % LOG_POLL_MAX;
  if (state.logCnt < LOG_POLL_MAX) state.logCnt++;
  Serial.println(buf);
}

void motorEnable(bool en) {
  digitalWrite(MOTOR_STBY, en ? HIGH : LOW);
}

void applyMotorPWM(int16_t left, int16_t right) {
  int lim = state.speedLimit;
  left  = constrain(left,  -lim, lim);
  right = constrain(right, -lim, lim);

  if (left >= 0) {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
    ledcWrite(PWM_CH_L, left);
  } else {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
    ledcWrite(PWM_CH_L, -left);
  }

  if (right >= 0) {
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, LOW);
    ledcWrite(PWM_CH_R, right);
  } else {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, HIGH);
    ledcWrite(PWM_CH_R, -right);
  }

  state.leftMotor  = left;
  state.rightMotor = right;
}

void setMotor(int16_t left, int16_t right) {
  motor.targetL = left;
  motor.targetR = right;
  motor.lastCmdTime = millis();
  motorEnable(true);

  // Apply trim
  if (left > 0) left += state.trimL;
  else if (left < 0) left -= state.trimL;
  if (right > 0) right += state.trimR;
  else if (right < 0) right -= state.trimR;
}

void stopMotors() {
  setMotor(0, 0);
}

void motorUpdate() {
  unsigned long now = millis();

  // Safety timeout
  if (now - motor.lastCmdTime > MOTOR_TIMEOUT && (motor.targetL != 0 || motor.targetR != 0)) {
    motor.targetL = 0;
    motor.targetR = 0;
  }

  // Ramp L
  if (motor.currentL < motor.targetL) {
    motor.currentL = min(motor.currentL + RAMP_STEP, motor.targetL);
  } else if (motor.currentL > motor.targetL) {
    motor.currentL = max(motor.currentL - RAMP_STEP, motor.targetL);
  }

  // Ramp R
  if (motor.currentR < motor.targetR) {
    motor.currentR = min(motor.currentR + RAMP_STEP, motor.targetR);
  } else if (motor.currentR > motor.targetR) {
    motor.currentR = max(motor.currentR - RAMP_STEP, motor.targetR);
  }

  applyMotorPWM(motor.currentL, motor.currentR);

  // Disable STBY when both motors are idle
  if (motor.currentL == 0 && motor.currentR == 0 && motor.targetL == 0 && motor.targetR == 0) {
    motorEnable(false);
  }
}

// ── Buzzer ──────────────────────────────────────────────────────────────────
void buzzerTone(uint16_t freq, uint16_t durationMs) {
  ledcWriteTone(BUZZER_PWM_CH, freq);
  delay(durationMs);
  ledcWriteTone(BUZZER_PWM_CH, 0);
}

void buzzerBeep() {
  buzzerTone(2000, 100);
}

void buzzerOk() {
  buzzerTone(1200, 80);
  delay(60);
  buzzerTone(1800, 80);
}

void buzzerErr() {
  buzzerTone(400, 200);
  delay(100);
  buzzerTone(300, 300);
}

// ── LED ─────────────────────────────────────────────────────────────────────
void updateLED() {
  unsigned long now = millis();
  if (now - lastLEDMs < 50) return;
  lastLEDMs = now;

  if (state.emergency) {
    digitalWrite(LED_PIN, (now / 120) % 2);
  } else if (state.behavior == "explore") {
    digitalWrite(LED_PIN, (now / 600) % 2);
  } else if (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, (now / 200) % 2);
  } else {
    digitalWrite(LED_PIN, HIGH);
  }
}

// ── Sensors ──────────────────────────────────────────────────────────────────
bool initVL53L0X() {
  pinMode(VL53L0X_XSHUT, OUTPUT);
  digitalWrite(VL53L0X_XSHUT, HIGH);
  delay(10);

  if (!tof.begin()) {
    // Retry with XSHUT toggle
    digitalWrite(VL53L0X_XSHUT, LOW);
    delay(5);
    digitalWrite(VL53L0X_XSHUT, HIGH);
    delay(10);
    if (!tof.begin()) return false;
  }
  tof.startRangeContinuous();
  applyVLConfig();
  return true;
}

uint16_t readDistance() {
  if (tof.isRangeComplete()) {
    VL53L0X_RangingMeasurementData_t measure;
    tof.readRangeResult(&measure);
    if (measure.RangeStatus != 4) {
      state.lastGoodDistance = measure.RangeMilliMeter;
      state.vlSignalRate = measure.SignalRateRtnMegaCps;
      state.vlAmbientRate = measure.AmbientRateMegaCps;
      state.vlValid = measure.SignalRateRtnMegaCps > VL_SIGNAL_OK_THRESHOLD
                      && measure.AmbientRateMegaCps < VL_AMBIENT_OK_THRESHOLD;
      return measure.RangeMilliMeter;
    }
  }
  return state.lastGoodDistance;
}

void applyVLConfig() {
  if (!state.sensorOk) return;
  uint32_t budget = constrain(state.vlTimingBudget, 20000U, 200000U);
  tof.setMeasurementTimingBudget(budget);
  tof.setOffset(state.vlOffset);
  logMsg("VL config: budget=%dus offset=%dmm", budget, state.vlOffset);
}

void autoVLTiming() {
  if (!state.sensorOk || !state.vlAuto) return;

  uint32_t target;
  if (state.behavior == "explore" && state.distance < state.safeDist * 3) {
    target = 20000; // high_speed near obstacles
  } else if (state.behavior == "explore") {
    target = 33000; // standard
  } else if (state.leftMotor == 0 && state.rightMotor == 0) {
    target = 200000; // high_accuracy when stationary
  } else {
    target = 33000; // standard in manual motion
  }

  if (target != state.vlTimingBudget) {
    state.vlTimingBudget = target;
    applyVLConfig();
  }
}

bool initMPU6050() {
  mpu.initialize();
  if (!mpu.testConnection()) return false;

  int devStatus = mpu.dmpInitialize();

  // Supply your own gyro offsets here, scaled to min/max
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus != 0) return false;

  mpu.CalibrateGyro(6);
  mpu.CalibrateAccel(6);

  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
  dmpReady = true;

  state.gyroOffsetX = mpu.getXGyroOffset();
  state.gyroOffsetY = mpu.getYGyroOffset();
  state.gyroOffset  = mpu.getZGyroOffset();

  return true;
}

void readIMU() {
  if (!dmpReady) return;

  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return;

  Quaternion q;
  VectorFloat gravity;
  float ypr[3];

  // DMP quaternion → yaw/pitch/roll
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  state.yaw   = ypr[0] * 180.0 / PI;
  state.pitch = ypr[1] * 180.0 / PI;
  state.roll  = ypr[2] * 180.0 / PI;

  // Raw gyro (LSB → deg/s, ±250dps = 131 LSB/(deg/s))
  int16_t gx, gy, gz;
  mpu.dmpGetGyro(&gx, &gy, &gz, fifoBuffer);
  state.gyroX = gx / 131.0;
  state.gyroY = gy / 131.0;
  state.gyroZ = gz / 131.0;

  // Raw accel for collision (LSB → m/s², ±2g = 16384 LSB/g)
  int16_t ax, ay, az;
  mpu.dmpGetAccel(&ax, &ay, &az, fifoBuffer);
  float amag = sqrt(ax*ax + ay*ay + az*az) / 16384.0 * 9.80665;
  state.accelMagnitude = amag;

  // Temperature
  state.temperature = mpu.getTemperature();

  unsigned long now = millis();

  // Collision detection
  if (amag > COLLISION_THRESHOLD && !state.collisionDetected) {
    state.collisionDetected = true;
    state.collisionTime = now;
    logMsg("COLLISION! accel=%.1f m/s^2", amag);
  } else if (state.collisionDetected && amag < COLLISION_THRESHOLD - 2) {
    state.collisionDetected = false;
  }

  // Tilt detection
  float tilt = sqrt(state.roll * state.roll + state.pitch * state.pitch);
  if (tilt > TILT_THRESHOLD_ON && !state.tiltDetected) {
    state.tiltDetected = true;
    state.emergency = true;
    stopMotors();
    logMsg("TILT! roll=%.1f pitch=%.1f", state.roll, state.pitch);
  } else if (state.tiltDetected && tilt < TILT_THRESHOLD_OFF) {
    state.tiltDetected = false;
    state.emergency = false;
    logMsg("Tilt cleared");
  }
}

// ── Servo sector scan ────────────────────────────────────────────────────────
void updateSectorScan() {
  if (state.behavior != "explore" || !state.sensorOk) {
    scanStep = SCAN_IDLE;
    return;
  }

  unsigned long now = millis();
  if (now - lastScanMs < 160) return;
  lastScanMs = now;

  switch (scanStep) {
    case SCAN_IDLE:
      servo.write(SECTOR_L);
      state.servoAng = SECTOR_L;
      scanStep = SCAN_L;
      break;
    case SCAN_L:
      state.sectors[0] = readDistance();
      servo.write(SECTOR_F);
      state.servoAng = SECTOR_F;
      scanStep = SCAN_F;
      break;
    case SCAN_F:
      state.sectors[1] = readDistance();
      servo.write(SECTOR_R);
      state.servoAng = SECTOR_R;
      scanStep = SCAN_R;
      break;
    case SCAN_R:
      state.sectors[2] = readDistance();
      servo.write(SECTOR_L);
      state.servoAng = SECTOR_L;
      scanStep = SCAN_L;
      break;
  }
}

// ── Behavior ─────────────────────────────────────────────────────────────────
void runExplore() {
  unsigned long now = millis();
  int16_t  spd = state.exploreSpd;
  int16_t  safe = state.safeDist;
  uint16_t d   = state.distance;

  switch (explorePhase) {
    case EXP_FWD: {
      // Steer toward more open side
      int l = state.sectors[0];
      int r = state.sectors[2];

      if (l < 0 && r < 0) {
        setMotor(spd, spd);
      } else if (r > l + 100) {
        setMotor(spd * 0.7, spd);
      } else if (l > r + 100) {
        setMotor(spd, spd * 0.7);
      } else {
        setMotor(spd, spd);
      }

      if (d > 0 && d < safe) {
        stopMotors();
        explorePhase = EXP_REV;
        phaseStart = now;
        logMsg("obstacle %d mm", d);
      }
      break;
    }
    case EXP_REV: {
      setMotor(-spd * 0.6, -spd * 0.6);
      if (now - phaseStart > 500) {
        stopMotors();
        explorePhase = EXP_TURN;
        phaseStart = now;
      }
      break;
    }
    case EXP_TURN: {
      int l = state.sectors[0];
      int r = state.sectors[2];
      if (l > r) {
        setMotor(-spd * 0.5, spd * 0.5);
      } else {
        setMotor(spd * 0.5, -spd * 0.5);
      }
      if (now - phaseStart > 700) {
        stopMotors();
        explorePhase = EXP_FWD;
      }
      break;
    }
  }
}

void updateBehavior() {
  if (state.emergency) {
    stopMotors();
    return;
  }

  // Auto-tune VL timing budget based on motion
  autoVLTiming();

  // Collision auto-stop in explore mode
  if (state.collisionDetected && state.behavior == "explore") {
    stopMotors();
    return;
  }

  // Safety stop (only if VL reading is valid)
  if (state.sensorOk && state.vlValid && state.distance > 0 && state.distance < state.safeDist) {
    if (state.behavior != "explore") {
      setMotor(0, 0);
    }
    return;
  }

  if (state.behavior == "explore") {
    runExplore();
  }
}

// ── Web Handlers ─────────────────────────────────────────────────────────────
void handleTelemetry() {
  state.uptime = millis() / 1000;
  state.rssi   = WiFi.RSSI();
  state.heap   = ESP.getFreeHeap();
  state.ssid   = WiFi.SSID();

  JsonDocument doc;
  char ipStr[16];
  sprintf(ipStr, "%d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1],
    WiFi.localIP()[2], WiFi.localIP()[3]);
  doc["ip"]        = ipStr;
  doc["mode"]      = state.emergency ? "emergency" : (state.behavior == "explore" ? "auto" : "manual");
  doc["behavior"]  = state.behavior;
  doc["speed"]     = (abs(state.leftMotor) + abs(state.rightMotor)) / 2;
  doc["left"]      = state.leftMotor;
  doc["right"]     = state.rightMotor;
  doc["distance"]  = state.distance;
  doc["safeDist"]  = state.safeDist;
  doc["servo"]     = state.servoAng;
  doc["speedLimit"] = state.speedLimit;
  doc["trimL"]     = state.trimL;
  doc["trimR"]     = state.trimR;
  doc["yaw"]       = state.yaw;
  doc["roll"]      = state.roll;
  doc["pitch"]     = state.pitch;
  doc["gyroX"]       = state.gyroX;
  doc["gyroY"]       = state.gyroY;
  doc["gyroZ"]       = state.gyroZ;
  doc["gyroOffsetX"] = state.gyroOffsetX;
  doc["gyroOffsetY"] = state.gyroOffsetY;
  doc["gyroOffset"]  = state.gyroOffset;
  doc["tiltDetected"]   = state.tiltDetected;
  doc["collisionDetected"] = state.collisionDetected;
  doc["accelMag"]    = state.accelMagnitude;
  doc["temp"]        = state.temperature;
  doc["vlSignalRate"]  = state.vlSignalRate;
  doc["vlAmbientRate"] = state.vlAmbientRate;
  doc["vlTimingBudget"] = (int)state.vlTimingBudget;
  doc["vlOffset"]     = state.vlOffset;
  doc["vlPreset"]     = state.vlPreset;
  doc["vlValid"]      = state.vlValid;
  doc["vlAuto"]       = state.vlAuto;

  JsonArray sec = doc["sectors"].to<JsonArray>();
  for (int i = 0; i < 3; i++) sec.add(state.sectors[i]);

  doc["rssi"]       = state.rssi;
  doc["heap"]       = (int)state.heap;
  doc["uptime"]     = (int)state.uptime;
  doc["ssid"]       = state.ssid;
  doc["sensor_ok"]  = state.sensorOk;
  doc["mpu_ok"]     = state.mpuOk;
  doc["emergency"]  = state.emergency;
  doc["exploreSpeed"] = state.exploreSpd;
  doc["version"]    = state.version;

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleCmd() {
  if (!server.hasArg("plain")) { server.send(400); return; }

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err) { server.send(400, "text/plain", err.c_str()); return; }

  bool changed = false;

  if (!doc["leftMotor"].isNull() || !doc["rightMotor"].isNull()) {
    int16_t l = doc["leftMotor"] | state.leftMotor;
    int16_t r = doc["rightMotor"] | state.rightMotor;
    setMotor(l, r);
    changed = true;
  }

  if (!doc["behavior"].isNull()) {
    state.behavior = doc["behavior"].as<String>();
    if (state.behavior != "explore") {
      explorePhase = EXP_FWD;
      scanStep = SCAN_IDLE;
      state.servoAng = 90;
      servo.write(90);
    }
    changed = true;
  }

  if (!doc["servo"].isNull()) {
    state.servoAng = constrain(doc["servo"].as<int>(), 0, 180);
    servo.write(state.servoAng);
    changed = true;
  }

  if (!doc["speedLimit"].isNull()) {
    state.speedLimit = constrain(doc["speedLimit"].as<int>(), 0, 255);
    changed = true;
  }

  if (!doc["trimL"].isNull()) {
    state.trimL = constrain(doc["trimL"].as<int>(), -50, 50);
    changed = true;
  }

  if (!doc["trimR"].isNull()) {
    state.trimR = constrain(doc["trimR"].as<int>(), -50, 50);
    changed = true;
  }

  if (!doc["safeDist"].isNull()) {
    state.safeDist = constrain(doc["safeDist"].as<int>(), 30, 2000);
    changed = true;
  }

  if (!doc["headingReset"].isNull()) {
    state.yaw = 0;
    changed = true;
  }

  bool vlChanged = false;
  if (!doc["vlAuto"].isNull()) {
    state.vlAuto = doc["vlAuto"].as<bool>();
    vlChanged = true;
  }
  if (!doc["vlPreset"].isNull()) {
    state.vlPreset = doc["vlPreset"].as<String>();
    if (state.vlPreset == "high_speed")      state.vlTimingBudget = 20000;
    else if (state.vlPreset == "standard")    state.vlTimingBudget = 33000;
    else if (state.vlPreset == "high_accuracy") state.vlTimingBudget = 200000;
    else if (state.vlPreset == "long_range")  state.vlTimingBudget = 200000;
    else state.vlPreset = "standard";
    state.vlAuto = false;
    vlChanged = true;
  }
  if (!doc["vlTimingBudget"].isNull()) {
    state.vlTimingBudget = constrain(doc["vlTimingBudget"].as<uint32_t>(), 20000U, 200000U);
    state.vlPreset = "custom";
    state.vlAuto = false;
    vlChanged = true;
  }
  if (!doc["vlOffset"].isNull()) {
    state.vlOffset = constrain(doc["vlOffset"].as<int>(), -100, 100);
    vlChanged = true;
  }
  if (vlChanged) {
    applyVLConfig();
    changed = true;
  }

  if (!doc["emergency"].isNull()) {
    state.emergency = doc["emergency"].as<bool>();
    if (state.emergency) stopMotors();
    changed = true;
  }

  if (!doc["exploreSpeed"].isNull()) {
    state.exploreSpd = constrain(doc["exploreSpeed"].as<int>(), 60, 255);
    changed = true;
  }

  if (changed) {
    logMsg("cmd OK");
    prefs.putInt("speedLimit",  state.speedLimit);
    prefs.putInt("safeDist",    state.safeDist);
    prefs.putInt("exploreSpd",  state.exploreSpd);
    prefs.putInt("trimL",       state.trimL);
    prefs.putInt("trimR",       state.trimR);
  }

  server.send(200, "text/plain", "ok");
}

void handleLogs() {
  String out;
  int cnt = state.logCnt;
  for (int i = 0; i < cnt; i++) {
    int idx = (state.logIdx - cnt + i + LOG_POLL_MAX) % LOG_POLL_MAX;
    out += state.logBuf[idx] + "\n";
  }
  state.logCnt = 0;
  server.send(200, "text/plain", out);
}

void handleRoot() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<title>KEI Robot</title><style>body{font-family:sans-serif;background:#0a0a0f;color:#e8e8ed;padding:20px;max-width:800px;margin:auto}"
    "h1{color:#f472b6}.nav{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:20px}"
    "a{display:block;padding:16px;background:rgba(255,255,255,.03);border:1px solid rgba(255,255,255,.06);border-radius:12px;text-decoration:none;color:#e8e8ed;text-align:center;font-weight:600}"
    "a:hover{background:rgba(244,114,182,.08);border-color:rgba(244,114,182,.2)}"
    ".sub{color:#52525b;font-size:12px;margin-top:4px}.v{color:#a855f7;font-size:14px}</style></head><body>"
    "<h1>KEI Robot</h1>"
    "<p class=v>v" + String(state.version) + "</p>"
    "<div class=nav>"
    "<a href='/telemetry'>Telemetry<span class=sub>JSON status</span></a>"
    "<a href='/config'>Konfigurasi<span class=sub>Settings</span></a>"
    "<a href='/diag'>Diagnosa<span class=sub>Sensors &amp; I2C</span></a>"
    "<a href='/update'>OTA Update<span class=sub>Firmware upload</span></a>"
    "<a href='/ping'>Ping<span class=sub>Health check</span></a>"
    "<a href='/version'>Version<span class=sub>Firmware info</span></a>"
    "</div></body></html>";
  server.send(200, "text/html", html);
}

void handleConfig() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>KEI Config</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>body{font-family:sans-serif;background:#111;color:#eee;padding:20px;max-width:600px;margin:auto}"
    "h1{color:#f472b6}label{display:block;margin:12px 0 4px}input{width:100%;padding:6px;background:#222;color:#eee;border:1px solid #333;border-radius:4px}"
    ".btn{background:#f472b6;color:#000;border:none;padding:10px 20px;border-radius:6px;font-weight:bold;cursor:pointer;margin-top:16px}"
    "</style></head><body><h1>Konfigurasi</h1>"
    "<form id=f><label>Safe Distance (mm)</label><input name=safeDist type=number value='" + String(state.safeDist) + "'>";
  html += "<label>Speed Limit</label><input name=speedLimit type=number min=0 max=255 value='" + String(state.speedLimit) + "'>";
  html += "<label>Explore Speed</label><input name=exploreSpeed type=number min=60 max=255 value='" + String(state.exploreSpd) + "'>";
  html += "<label>Trim Left</label><input name=trimL type=number min=-50 max=50 value='" + String(state.trimL) + "'>";
  html += "<label>Trim Right</label><input name=trimR type=number min=-50 max=50 value='" + String(state.trimR) + "'>";
  html += "<button class=btn type=submit>Simpan</button></form>"
    "<script>document.getElementById('f').onsubmit=async function(e){e.preventDefault();"
    "var d=Object.fromEntries(new FormData(this));await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({"
    "safeDist:+d.safeDist,speedLimit:+d.speedLimit,exploreSpeed:+d.exploreSpeed,trimL:+d.trimL,trimR:+d.trimR})});"
    "alert('Disimpan!');}</script></body></html>";
  server.send(200, "text/html", html);
}

void handleDiag() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>KEI Diag</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<meta http-equiv='refresh' content='2'>"
    "<style>body{font-family:monospace;background:#0a0a0f;color:#e8e8ed;padding:20px}"
    ".ok{color:#4ade80}.err{color:#f87171}</style></head><body><h2>Diagnostik</h2>"
    "<pre>";
  html += "WiFi       : " + WiFi.SSID() + " (" + WiFi.localIP().toString() + ")\n";
  html += "RSSI       : " + String(WiFi.RSSI()) + " dBm\n";
  html += "Heap       : " + String(ESP.getFreeHeap()) + " B\n";
  html += "CPU Freq   : " + String(getCpuFrequencyMhz()) + " MHz\n";
  html += "Uptime     : " + String(millis() / 1000) + " s\n";
  html += "Mode       : " + (state.emergency ? "EMERGENCY" : state.behavior) + "\n";
  html += "Speed Limit: " + String(state.speedLimit) + "\n";
  html += "Safe Dist  : " + String(state.safeDist) + " mm\n";
  html += "Trim L/R   : " + String(state.trimL) + " / " + String(state.trimR) + "\n";
  html += "VL53L0X    : " + String(state.sensorOk ? "OK" : "ERR") + "\n";
  html += "VL Preset  : " + state.vlPreset + "\n";
  html += "VL Timing  : " + String(state.vlTimingBudget) + " us\n";
  html += "VL Offset  : " + String(state.vlOffset) + " mm\n";
  html += "VL Signal  : " + String(state.vlSignalRate, 3) + " Mcps\n";
  html += "VL Ambient : " + String(state.vlAmbientRate, 3) + " Mcps\n";
  html += "VL Valid   : " + String(state.vlValid ? "YES" : "NO") + "\n";
  html += "VL Auto    : " + String(state.vlAuto ? "ON" : "OFF") + "\n";
  html += "MPU6050    : " + String(state.mpuOk ? "OK" : "ERR") + "\n";
  html += "Yaw        : " + String(state.yaw, 1) + "\n";
  html += "Roll       : " + String(state.roll, 1) + "\n";
  html += "Pitch      : " + String(state.pitch, 1) + "\n";
  html += "Gyro X     : " + String(state.gyroX, 1) + " deg/s\n";
  html += "Gyro Y     : " + String(state.gyroY, 1) + " deg/s\n";
  html += "Gyro Z     : " + String(state.gyroZ, 1) + " deg/s\n";
  html += "G Off X/Y/Z: " + String(state.gyroOffsetX, 2) + "/" + String(state.gyroOffsetY, 2) + "/" + String(state.gyroOffset, 2) + " deg/s\n";
  html += "Tilt       : " + String(state.tiltDetected ? "YES" : "no") + "\n";
  html += "Collision  : " + String(state.collisionDetected ? "YES" : "no") + "\n";
  html += "Accel Mag  : " + String(state.accelMagnitude, 1) + " m/s^2\n";
  html += "Temp       : " + String(state.temperature, 1) + " C\n";
  html += "Distance   : " + (state.distance == 0 ? "N/A" : String(state.distance) + " mm") + "\n";
  html += "Sectors    : " + String(state.sectors[0]) + "/" + String(state.sectors[1]) + "/" + String(state.sectors[2]) + " mm\n";
  html += "Servo      : " + String(state.servoAng) + "\n";
  html += "Log Count  : " + String(state.logCnt) + "\n";
  html += "Firmware   : v" + String(state.version) + "\n";
  html += "</pre><a href=/>Dashboard</a></body></html>";
  server.send(200, "text/html", html);
}

void handlePing() {
  server.send(200, "text/plain", "pong");
}

void handleVersion() {
  JsonDocument doc;
  doc["version"] = state.version;
  doc["board"]   = "esp32dev";
  doc["build"]   = __DATE__ " " __TIME__;
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleUpdate() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>KEI OTA</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>body{font-family:sans-serif;background:#111;color:#eee;padding:20px;max-width:600px;margin:auto}"
    "h1{color:#f472b6}input,button{width:100%;padding:10px;margin:8px 0;border-radius:6px}"
    "button{background:#f472b6;color:#000;font-weight:bold;border:none;cursor:pointer}"
    "</style></head><body><h1>OTA Update</h1>"
    "<p>Upload firmware .bin via web:</p>"
    "<form method=POST action='/upload' enctype=multipart/form-data>"
    "<input type=file name=firmware accept='.bin'>"
    "<button type=submit>Upload & Flash</button></form>"
    "<p style='color:#52525b;font-size:12px'>Atau gunakan ArduinoOTA dari PlatformIO</p></body></html>";
  server.send(200, "text/html", html);
}

void handleUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    logMsg("OTA upload: %s", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Update.printError(Serial);
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (Update.end(true)) {
      logMsg("OTA sukses (%u B), rebooting...", upload.totalSize);
      server.send(200, "text/plain", "OK. Rebooting...");
      delay(500);
      ESP.restart();
    } else {
      Update.printError(Serial);
    }
  }
}

// ── Setup ────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  logMsg("KEI Robot v%s starting...", state.version);

  // Power save: 80 MHz CPU
  setCpuFrequencyMhz(80);

  // LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // STBY
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, LOW);

  // Motor pins
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);

  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(MOTOR_L_PWM, PWM_CH_L);
  ledcAttachPin(MOTOR_R_PWM, PWM_CH_R);
  stopMotors();

  // Servo
  servo.attach(SERVO_PIN);
  servo.write(90);

  // Buzzer
  ledcSetup(BUZZER_PWM_CH, BUZZER_PWM_FREQ, PWM_RES);
  ledcAttachPin(BUZZER_PIN, BUZZER_PWM_CH);

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // VL53L0X
  state.sensorOk = initVL53L0X();
  logMsg("VL53L0X: %s", state.sensorOk ? "OK" : "FAIL");

  // MPU6050
  state.mpuOk = initMPU6050();
  logMsg("MPU6050: %s", state.mpuOk ? "OK" : "FAIL");

  if (state.sensorOk && state.mpuOk) {
    buzzerOk();
  } else {
    buzzerErr();
  }

  // Preferences
  prefs.begin("kei", false);
  state.speedLimit = prefs.getInt("speedLimit", DEFAULT_SPEED_LIMIT);
  state.safeDist   = prefs.getInt("safeDist", DEFAULT_SAFE_DIST_MM);
  state.exploreSpd = prefs.getInt("exploreSpd", DEFAULT_EXPLORE_SPEED);
  state.trimL      = prefs.getInt("trimL", 0);
  state.trimR      = prefs.getInt("trimR", 0);
  logMsg("Preferences loaded");

  // WiFi
  WiFi.mode(WIFI_STA);
#ifdef USE_STATIC_IP
  WiFi.config(STATIC_IP, GATEWAY, SUBNET);
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  logMsg("WiFi connecting to %s...", WIFI_SSID);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    logMsg("WiFi OK — IP: %s", WiFi.localIP().toString().c_str());
  } else {
    logMsg("WiFi FAIL — starting AP");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("KEI-Robot", "kei12345");
    logMsg("AP IP: %s", WiFi.softAPIP().toString().c_str());
  }

  // Modem sleep
  WiFi.setSleep(true);

  // mDNS
  if (MDNS.begin("kei")) {
    MDNS.addService("http", "tcp", 80);
    logMsg("mDNS: kei.local");
  }

  // ArduinoOTA
  ArduinoOTA.setHostname("kei");
  ArduinoOTA.begin();
  logMsg("ArduinoOTA ready");

  // Web routes
  server.on("/",         handleRoot);
  server.on("/telemetry", handleTelemetry);
  server.on("/cmd",      HTTP_POST, handleCmd);
  server.on("/logs",     handleLogs);
  server.on("/config",   handleConfig);
  server.on("/diag",     handleDiag);
  server.on("/ping",     handlePing);
  server.on("/version",  handleVersion);
  server.on("/update",   HTTP_GET, handleUpdate);
  server.on("/upload",   HTTP_POST, []() {}, handleUpload);

  server.begin();
  logMsg("HTTP server started on port 80");

  buzzerBeep();
}

// ── Loop ─────────────────────────────────────────────────────────────────────
void loop() {
  server.handleClient();
  ArduinoOTA.handle();

  unsigned long now = millis();

  // Motor ramp + safety timeout
  if (now - lastMotorMs >= RAMP_STEP_MS) {
    lastMotorMs = now;
    motorUpdate();
  }

  // Telemetry sensors
  if (now - lastTelemetry >= TELEMETRY_INTERVAL_MS) {
    lastTelemetry = now;

    if (state.sensorOk) {
      state.distance = readDistance();
    }

    if (state.mpuOk) {
      readIMU();
    }
  }

  // Servo sector scan in explore mode
  updateSectorScan();

  // Behavior
  updateBehavior();

  // LED
  updateLED();
}
