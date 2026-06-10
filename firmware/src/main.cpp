#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <config.h>

// Sensors
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Servo
#include <ESP32Servo.h>

// ── Objects ──────────────────────────────────────────────────────────────────
WebServer server(80);
VL53L0X tof;
Adafruit_MPU6050 mpu;
Servo servo;

// ── State ────────────────────────────────────────────────────────────────────
struct {
  // Motor
  int16_t leftMotor  = 0;
  int16_t rightMotor = 0;
  int16_t speedLimit = DEFAULT_SPEED_LIMIT;

  // Behavior
  String  behavior   = "stop";    // stop | explore | ...
  bool    emergency  = false;

  // Servo
  uint8_t servoAng   = 90;

  // Sensor readings
  uint16_t distance  = 0;
  int16_t  sectors[3] = {-1, -1, -1};   // L, F, R (mm)
  float    yaw   = 0;
  float    roll  = 0;
  float    pitch = 0;
  float    gyroZ = 0;

  // Tuning
  uint16_t safeDist   = DEFAULT_SAFE_DIST_MM;
  uint8_t  exploreSpd = DEFAULT_EXPLORE_SPEED;

  // IMU calibration offset
  float    yawOffset  = 0;

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
} state;

unsigned long lastTelemetry = 0;
unsigned long lastServoSweep = 0;
int servoSweepDir = 1;

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

void setMotor(int16_t left, int16_t right) {
  // Clamp & apply speed limit
  int16_t lim = state.speedLimit;
  left  = constrain(left,  -lim, lim);
  right = constrain(right, -lim, lim);

  // Left motor
  if (left >= 0) {
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, LOW);
  } else {
    digitalWrite(MOTOR_L_IN1, LOW);
    digitalWrite(MOTOR_L_IN2, HIGH);
    left = -left;
  }
  ledcWrite(PWM_CH_L, left);

  // Right motor
  if (right >= 0) {
    digitalWrite(MOTOR_R_IN1, HIGH);
    digitalWrite(MOTOR_R_IN2, LOW);
  } else {
    digitalWrite(MOTOR_R_IN1, LOW);
    digitalWrite(MOTOR_R_IN2, HIGH);
    right = -right;
  }
  ledcWrite(PWM_CH_R, right);

  state.leftMotor  = (left >= 0 ? left : -left) * (left >= 0 ? 1 : -1);
  state.rightMotor = (right >= 0 ? right : -right) * (right >= 0 ? 1 : -1);
}

void stopMotors() {
  setMotor(0, 0);
}

// ── Sensors ──────────────────────────────────────────────────────────────────
bool initVL53L0X() {
  if (!tof.init()) return false;
  tof.setTimeout(50);
  tof.startContinuous();
  return true;
}

uint16_t readDistance() {
  uint16_t d = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred()) return 0xFFFF;
  return d;
}

bool initMPU6050() {
  if (!mpu.begin()) return false;
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  return true;
}

void readIMU() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Roll & pitch from accelerometer (degrees)
  state.roll  = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  state.pitch = atan2(-a.acceleration.x,
    sqrt(a.acceleration.y * a.acceleration.y + a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  // Gyro Z (degrees/sec) — integrate for yaw
  state.gyroZ = g.gyro.z * 180.0 / PI;
  static unsigned long lastImuMs = 0;
  unsigned long now = millis();
  if (lastImuMs) {
    float dt = (now - lastImuMs) / 1000.0;
    state.yaw += g.gyro.z * dt * 180.0 / PI;
  }
  lastImuMs = now;
}

// ── Behavior ─────────────────────────────────────────────────────────────────
void runExplore() {
  // Simple obstacle-avoidance: read distance, turn if too close
  uint16_t d = readDistance();
  state.distance = d;

  // Sweep servo to scan
  unsigned long now = millis();
  if (now - lastServoSweep > 60) {
    lastServoSweep = now;
    state.servoAng += servoSweepDir;
    if (state.servoAng >= 160) servoSweepDir = -1;
    if (state.servoAng <= 20)  servoSweepDir = 1;
    servo.write(state.servoAng);
  }

  // Read sector distances from 3 servo positions
  // Simplified: use single distance as forward, simulate L/R
  int16_t safe = state.safeDist;
  state.sectors[0] = d + 50;  // simulated left
  state.sectors[1] = d;       // forward
  state.sectors[2] = d + 30;  // simulated right

  if (d > 0 && d < safe) {
    // Obstacle — turn
    if (state.servoAng > 90) {
      setMotor(state.exploreSpd * 0.6, -state.exploreSpd * 0.6);
    } else {
      setMotor(-state.exploreSpd * 0.6, state.exploreSpd * 0.6);
    }
  } else {
    setMotor(state.exploreSpd, state.exploreSpd);
  }
}

void updateBehavior() {
  if (state.emergency) {
    stopMotors();
    return;
  }
  if (state.behavior == "explore") {
    runExplore();
  } else {
    // Manual mode — motors are set via POST /cmd directly
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
  doc["ip"]       = ipStr;
  doc["mode"]     = state.emergency ? "emergency" : (state.behavior == "explore" ? "auto" : "manual");
  doc["behavior"] = state.behavior;
  doc["speed"]    = (abs(state.leftMotor) + abs(state.rightMotor)) / 2;
  doc["left"]     = state.leftMotor;
  doc["right"]    = state.rightMotor;
  doc["distance"] = (state.distance == 0xFFFF) ? -1 : (int)state.distance;
  doc["safeDist"] = state.safeDist;
  doc["servo"]    = state.servoAng;
  doc["speedLimit"] = state.speedLimit;
  doc["yaw"]      = state.yaw;
  doc["roll"]     = state.roll;
  doc["pitch"]    = state.pitch;
  doc["gyroZ"]    = state.gyroZ;

  JsonArray sec = doc["sectors"].to<JsonArray>();
  for (int i = 0; i < 3; i++) sec.add(state.sectors[i]);

  doc["rssi"]   = state.rssi;
  doc["heap"]   = (int)state.heap;
  doc["uptime"] = (int)state.uptime;
  doc["ssid"]   = state.ssid;
  doc["sensor_ok"] = state.sensorOk;
  doc["mpu_ok"]    = state.mpuOk;
  doc["emergency"] = state.emergency;
  doc["exploreSpeed"] = state.exploreSpd;

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
      // reset servo to center when leaving explore
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

  if (!doc["safeDist"].isNull()) {
    state.safeDist = constrain(doc["safeDist"].as<int>(), 30, 2000);
    changed = true;
  }

  if (!doc["headingReset"].isNull()) {
    state.yawOffset = state.yaw;
    state.yaw = 0;
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

  if (changed) logMsg("cmd OK");
  server.send(200, "text/plain", "ok");
}

void handleLogs() {
  // Return accumulated logs since last read, then clear
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
  // Serve index.html from SPIFFS — for now redirect to the web dashboard
  server.send(200, "text/html",
    "<!DOCTYPE html><html><head><meta charset='UTF-8'><meta http-equiv='refresh' content='0;url=/'>"
    "<title>KEI Robot</title></head><body><p>KEI Robot — <a href='/'>Dashboard</a></p></body></html>");
}

void handleConfig() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>KEI Config</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>body{font-family:sans-serif;background:#111;color:#eee;padding:20px;max-width:600px;margin:auto}"
    "h1{color:#f472b6}label{display:block;margin:12px 0 4px}input{width:100%;padding:6px;background:#222;color:#eee;border:1px solid #333;border-radius:4px}"
    ".btn{background:#f472b6;color:#000;border:none;padding:10px 20px;border-radius:6px;font-weight:bold;cursor:pointer;margin-top:16px}"
    "</style></head><body><h1>\u2699 Konfigurasi</h1>"
    "<form id=f><label>WiFi SSID</label><input name=ssid value='" WIFI_SSID "' disabled>";
  html += "<label>Safe Distance (mm)</label><input name=safeDist type=number value='" + String(state.safeDist) + "'>";
  html += "<label>Speed Limit</label><input name=speedLimit type=number min=0 max=255 value='" + String(state.speedLimit) + "'>";
  html += "<label>Explore Speed</label><input name=exploreSpeed type=number min=60 max=255 value='" + String(state.exploreSpd) + "'>";
  html += "<button class=btn type=submit>Simpan</button></form>"
    "<script>document.getElementById('f').onsubmit=async function(e){e.preventDefault();"
    "var d=Object.fromEntries(new FormData(this));await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({"
    "safeDist:+d.safeDist,speedLimit:+d.speedLimit,exploreSpeed:+d.exploreSpeed})});"
    "alert('Disimpan!');}</script></body></html>";
  server.send(200, "text/html", html);
}

void handleDiag() {
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>KEI Diag</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<meta http-equiv='refresh' content='2'>"
    "<style>body{font-family:monospace;background:#0a0a0f;color:#e8e8ed;padding:20px}"
    ".ok{color:#4ade80}.err{color:#f87171}</style></head><body><h2>🔍 Diagnostik</h2>"
    "<pre>";
  html += "WiFi       : " + WiFi.SSID() + " (" + WiFi.localIP().toString() + ")\n";
  html += "RSSI       : " + String(WiFi.RSSI()) + " dBm\n";
  html += "Heap       : " + String(ESP.getFreeHeap()) + " B\n";
  html += "Uptime     : " + String(millis() / 1000) + " s\n";
  html += "Mode       : " + (state.emergency ? "EMERGENCY" : state.behavior) + "\n";
  html += "Speed Limit: " + String(state.speedLimit) + "\n";
  html += "Safe Dist  : " + String(state.safeDist) + " mm\n";
  html += "VL53L0X    : " + String(state.sensorOk ? "OK" : "ERR") + "\n";
  html += "MPU6050    : " + String(state.mpuOk ? "OK" : "ERR") + "\n";
  html += "Yaw        : " + String(state.yaw, 1) + "\u00b0\n";
  html += "Roll       : " + String(state.roll, 1) + "\u00b0\n";
  html += "Pitch      : " + String(state.pitch, 1) + "\u00b0\n";
  html += "Gyro Z     : " + String(state.gyroZ, 1) + "\u00b0/s\n";
  html += "Distance   : " + (state.distance == 0xFFFF ? "ERR" : String(state.distance) + " mm") + "\n";
  html += "Servo      : " + String(state.servoAng) + "\u00b0\n";
  html += "Log Count  : " + String(state.logCnt) + "\n";
  html += "</pre><a href=/>← Dashboard</a></body></html>";
  server.send(200, "text/html", html);
}

void handleUpdate() {
  if (server.method() == HTTP_POST) {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      logMsg("OTA update: %s", upload.filename.c_str());
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
    return;
  }
  String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>KEI OTA</title>"
    "<meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>body{font-family:sans-serif;background:#111;color:#eee;padding:20px;max-width:600px;margin:auto}"
    "h1{color:#f472b6}input,button{width:100%;padding:10px;margin:8px 0;border-radius:6px}"
    "button{background:#f472b6;color:#000;font-weight:bold;border:none;cursor:pointer}"
    "</style></head><body><h1>⬆ OTA Update</h1>"
    "<form method=POST enctype=multipart/form-data>"
    "<input type=file name=update accept='.bin'>"
    "<button type=submit>Upload & Flash</button></form></body></html>";
  server.send(200, "text/html", html);
}

// ── Setup & Loop ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  logMsg("KEI Robot v1.0 starting...");

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

  // I2C
  Wire.begin(I2C_SDA, I2C_SCL);

  // VL53L0X
  state.sensorOk = initVL53L0X();
  logMsg("VL53L0X: %s", state.sensorOk ? "OK" : "FAIL");

  // MPU6050
  state.mpuOk = initMPU6050();
  logMsg("MPU6050: %s", state.mpuOk ? "OK" : "FAIL");

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

#ifdef USE_STATIC_IP
  WiFi.config(STATIC_IP, GATEWAY, SUBNET);
#endif

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

  // mDNS
  if (MDNS.begin("kei")) {
    MDNS.addService("http", "tcp", 80);
    logMsg("mDNS: kei.local");
  }

  // Web routes
  server.on("/", handleRoot);
  server.on("/telemetry", handleTelemetry);
  server.on("/cmd", HTTP_POST, handleCmd);
  server.on("/logs", handleLogs);
  server.on("/config", handleConfig);
  server.on("/diag", handleDiag);
  server.on("/update", HTTP_GET, handleUpdate);
  server.on("/update", HTTP_POST, []() {}, handleUpdate);

  server.begin();
  logMsg("HTTP server started on port 80");
}

void loop() {
  server.handleClient();

  // Read sensors
  unsigned long now = millis();
  if (now - lastTelemetry > TELEMETRY_INTERVAL_MS) {
    lastTelemetry = now;

    // Distance
    if (state.sensorOk) {
      state.distance = readDistance();
    }

    // IMU
    if (state.mpuOk) {
      readIMU();
    }
  }

  // Run behavior
  updateBehavior();
}
