// ============================================================
//  SmartAlignOra — ESP32-C3 SuperMini
//  Board   : ESP32-C3 SuperMini
//  Sensor  : MPU6500 (sold as MPU6050) — WHO_AM_I = 0x70
//            SDA → GPIO 8  |  SCL → GPIO 9
//  Motor   : Vibration motor → GPIO 3 (PWM)
//  Library : MPU6050_light by rfetick
//            (works with MPU6500 clones, no WHO_AM_I restriction)
// ============================================================

#include <MPU6050_light.h>   // ← replaces Adafruit_MPU6050
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>

bool needCalibration = false;
bool isCalibrated    = false;

volatile bool deviceConnected     = false;
volatile bool previouslyConnected = false;

// ── PWM / Vibration ──────────────────────────────────────────
#define VIBRATION_PIN  3
#define PWM_FREQ       1000
#define PWM_RESOLUTION 8

int vibrationIntensity = 127;

int  currentPattern = 0;
bool patternRunning = false;
unsigned long patternTimer = 0;
int  patternStep   = 0;

// ── BLE UUIDs ────────────────────────────────────────────────
#define SERVICE_UUID      "a32be81d-570e-4ad9-bf2a-64fdfe3db515"
#define WRITE_CHAR_UUID   "c6b0278a-f9b5-4306-8eee-5d74d746bcc3"
#define READ_CHAR_UUID    "f53d0832-1177-479b-8046-1b76534390cb"
#define NOTIFY_CHAR_UUID  "b3af0550-1ba9-4efb-aac7-14287a527e06"

BLEServer         *pServer;
BLECharacteristic *writeChar;
BLECharacteristic *readChar;
BLECharacteristic *notifyChar;

bool     timeSynced  = false;
uint32_t baseEpoch   = 0;
uint32_t baseMillis  = 0;

unsigned long connectTime = 0;
const unsigned long HANDSHAKE_TIMEOUT = 10000;

// ── MPU object ───────────────────────────────────────────────
MPU6050 mpu(Wire);   // pass Wire instance directly

// ── Kalman state ─────────────────────────────────────────────
double Q_angle   = 0.001;
double Q_bias    = 0.003;
double R_measure = 0.03;

double angle  = 0, bias  = 0;
double angleR = 0, biasR = 0;
float  smoothMag = 9.8;

double P[2][2]  = {{0, 0}, {0, 0}};
double PR[2][2] = {{0, 0}, {0, 0}};
double dt;
unsigned long lastTime      = 0;
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 20; // 50 Hz

double pitchOffset = 0;
double rollOffset  = 0;

// Forward declaration
double Kalman_filter(double accelAngle, double gyroRate,
                     double &angle, double &bias, double P[2][2]);

/* ═══════════════════ WRITE CALLBACK ════════════════════════ */
class TimeWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {

    String value = pChar->getValue();
    Serial.print("BLE Received Command: ");
    Serial.println(value);
    if (value.length() == 0) return;

    if (!timeSynced) {
      uint32_t receivedTime = value.toInt();
      if (receivedTime < 1600000000) {
        Serial.println("Invalid timestamp received");
        return;
      }
      baseEpoch  = receivedTime;
      baseMillis = millis();
      timeSynced = true;
      Serial.print("Timestamp received: "); Serial.println(baseEpoch);
      Serial.println("Handshake successful!");
      return;
    }

    if (value == "C1") {
      needCalibration = true;
      Serial.println("Calibration triggered by app (C1)");
      return;
    }

    if (value.startsWith("S")) {
      int patNum = value.substring(1).toInt();
      if (patNum >= 1 && patNum <= 6) {
        ledcWrite(VIBRATION_PIN, 0);
        currentPattern = patNum;
        patternRunning = true;
        patternStep    = 0;
        patternTimer   = millis();
        Serial.print("Pattern started: S"); Serial.println(patNum);
      } else {
        Serial.println("Invalid pattern number");
      }
      return;
    }

    if (value.startsWith("V")) {
      int percent = constrain(value.substring(1).toInt(), 0, 100);
      vibrationIntensity = map(percent, 0, 100, 0, 255);
      Serial.print("Vibration intensity set to ");
      Serial.print(percent); Serial.println("%");
      if (percent == 0) {
        ledcWrite(VIBRATION_PIN, 0);
        Serial.println("Motor stopped (V0 received)");
        return;
      }
      if (patternRunning)
        Serial.println("Intensity updated, applies on next pattern cycle");
      return;
    }

    if (value == "0") {
      patternRunning = false;
      patternStep    = 0;
      currentPattern = 0;
      ledcWrite(VIBRATION_PIN, 0);
      Serial.println("Motor OFF");
      return;
    }
  }
};

/* ═══════════════════ SERVER CALLBACKS ══════════════════════ */
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) {
    deviceConnected = true;
    timeSynced      = false;
    connectTime     = millis();
    Serial.println("Client connected, waiting for timestamp...");
  }
  void onDisconnect(BLEServer* server) override {
    vibrationIntensity = 127;
    patternRunning     = false;
    patternStep        = 0;
    currentPattern     = 0;
    ledcWrite(VIBRATION_PIN, 0);
    deviceConnected     = false;
    timeSynced          = false;
    isCalibrated        = false;
    previouslyConnected = true;
    Serial.println("[BLE] Client disconnected — motor stopped");
  }
};

void restartAdvertising() {
  delay(200);
  BLEAdvertising *pAdv = BLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setScanResponse(true);
  pAdv->setMinPreferred(0x06);
  pAdv->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("[BLE] Advertising restarted");
}

/* ── Calibration ─────────────────────────────────────────────*/
void calibrateMPU() {
  Serial.println("Calibrating... Keep device steady");

  double sumPitch = 0, sumRoll = 0;
  const int samples = 500;
  smoothMag = 9.8;

  for (int i = 0; i < samples; i++) {
    mpu.update();  // MPU6050_light uses mpu.update() to refresh data

    float ax = mpu.getAccX();  // m/s² already
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();

    sumPitch += atan2(-ax, sqrt(ay*ay + az*az)) * 180.0 / PI;
    sumRoll  += atan2( ay, sqrt(ax*ax + az*az)) * 180.0 / PI;
    delay(5);
  }

  pitchOffset = sumPitch / samples;
  rollOffset  = sumRoll  / samples;

  Serial.print("Pitch Offset: "); Serial.println(pitchOffset);
  Serial.print("Roll Offset:  "); Serial.println(rollOffset);
}

/* ═══════════════════ SETUP ══════════════════════════════════ */
void setup() {
  Serial.begin(115200);
  delay(2000); // USB CDC stabilization for ESP32-C3

  Serial.println("SmartAlignOra — ESP32-C3 SuperMini");
  Serial.println("Chip: MPU6500 (WHO_AM_I=0x70)");
  Serial.println("Booting...");

  // I2C — confirmed by scanner: SDA=GPIO8, SCL=GPIO9
  Wire.begin(8, 9);
  Wire.setClock(400000); // 400kHz fast mode
  delay(200);

  // PWM
  ledcAttach(VIBRATION_PIN, PWM_FREQ, PWM_RESOLUTION);
  ledcWrite(VIBRATION_PIN, 0);

  // MPU init — MPU6050_light does NOT check WHO_AM_I strictly
  Serial.println("Initializing MPU6500...");
  byte status = mpu.begin();
  Serial.print("MPU6050_light begin status: ");
  Serial.println(status);

  if (status != 0) {
    Serial.println("Failed to initialize MPU! Check wiring.");
    while (1) { delay(500); }
  }

  // Configure ranges via raw register writes
  // (MPU6500 register map identical to MPU6050 for these)
  // Accel: ±8g  → register 0x1C, value 0x10
  // Gyro:  ±500 → register 0x1B, value 0x08
  // DLPF:  21Hz → register 0x1A, value 0x04
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); Wire.write(0x10); // Accel ±8g
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1B); Wire.write(0x08); // Gyro ±500 deg/s
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1A); Wire.write(0x04); // DLPF ~21Hz
  Wire.endTransmission();

  Serial.println("MPU6500 Ready!");
  lastTime = millis();

  // BLE
  BLEDevice::init("SmartAlignOra");
  BLEDevice::setPower(ESP_PWR_LVL_P9);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *service = pServer->createService(SERVICE_UUID);

  writeChar = service->createCharacteristic(WRITE_CHAR_UUID,
                BLECharacteristic::PROPERTY_WRITE);
  writeChar->setCallbacks(new TimeWriteCallback());

  readChar = service->createCharacteristic(READ_CHAR_UUID,
               BLECharacteristic::PROPERTY_READ);
  readChar->setValue("ESP32-C3 Ready");

  notifyChar = service->createCharacteristic(NOTIFY_CHAR_UUID,
                 BLECharacteristic::PROPERTY_NOTIFY);
  notifyChar->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMaxPreferred(0x12);
  advertising->start();

  Serial.println("================================");
  Serial.println("BLE Advertising Started");
  Serial.println("Device Name: SmartAlignOra");
  Serial.println("Waiting for phone connection...");
  Serial.println("================================");
}

/* ═══════════════════ VIBRATION PATTERNS ═════════════════════ */
void runVibrationPattern() {
  if (!patternRunning) return;
  unsigned long now = millis();

  switch (currentPattern) {
    case 1: // Short (200ms ON, 300ms OFF)
      if      (patternStep == 0)                              { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 1 && now - patternTimer >= 200) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 2 && now - patternTimer >= 300) { patternStep = 0; }
      break;

    case 2: // Medium (500ms ON, 300ms OFF)
      if      (patternStep == 0)                              { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 1 && now - patternTimer >= 500) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 2 && now - patternTimer >= 300) { patternStep = 0; }
      break;

    case 3: // Long (1000ms ON, 300ms OFF)
      if      (patternStep == 0)                               { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 1 && now - patternTimer >= 1000) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 2 && now - patternTimer >= 300)  { patternStep = 0; }
      break;

    case 4: // Tuk-Tuk
      if      (patternStep == 0)                              { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 1 && now - patternTimer >= 150) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 2 && now - patternTimer >= 100) { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 3 && now - patternTimer >= 150) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 4 && now - patternTimer >= 400) { patternStep = 0; }
      break;

    case 5: // Knock-Knock
      if      (patternStep == 0)                              { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 1 && now - patternTimer >= 200) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 2 && now - patternTimer >= 300) { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 3 && now - patternTimer >= 200) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 4 && now - patternTimer >= 500) { patternStep = 0; }
      break;

    case 6: // Heartbeat
      if      (patternStep == 0)                              { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 1 && now - patternTimer >= 100) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 2 && now - patternTimer >= 100) { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 3 && now - patternTimer >= 100) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 4 && now - patternTimer >= 500) { patternStep = 0; }
      break;

    default:
      patternRunning = false;
      patternStep    = 0;
      ledcWrite(VIBRATION_PIN, 0);
      break;
  }
}

/* ═══════════════════ LOOP ═══════════════════════════════════ */
void loop() {

  if (!deviceConnected && previouslyConnected) {
    previouslyConnected = false;
    restartAdvertising();
    return;
  }
  if (!deviceConnected) { delay(100); return; }

  if (!timeSynced) {
    if (millis() - connectTime > HANDSHAKE_TIMEOUT) {
      Serial.println("Handshake failed. Disconnecting...");
      pServer->disconnect(0);
      delay(100);
    }
    return;
  }

  if (needCalibration) {
    calibrateMPU();
    angle  = 0; bias  = 0;
    angleR = 0; biasR = 0;
    P[0][0]  = P[0][1]  = P[1][0]  = P[1][1]  = 0;
    PR[0][0] = PR[0][1] = PR[1][0] = PR[1][1] = 0;
    lastTime        = millis();
    lastPrintTime   = millis();
    needCalibration = false;
    isCalibrated    = true;
    return;
  }

  runVibrationPattern();

  // ── Read sensor via MPU6050_light ────────────────────────────
  mpu.update();

  // getAcc returns m/s², getGyro returns deg/s
  float ax = mpu.getAccX();
  float ay = mpu.getAccY();
  float az = mpu.getAccZ();
  float gx = mpu.getGyroX(); // deg/s
  float gy = mpu.getGyroY();
  float gz = mpu.getGyroZ();

  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  if (dt <= 0 || dt > 0.1) dt = 0.02;
  lastTime = now;

  // Accel angles (full ±180°)
  double accelPitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180.0/PI - pitchOffset;
  double accelRoll  = atan2( ay, sqrt(ax*ax + az*az)) * 180.0/PI - rollOffset;

  // Gyro deadband
  double gyroRateY = (abs(gy) < 1.0) ? 0.0 : (double)gy;
  double gyroRateX = (abs(gx) < 1.0) ? 0.0 : (double)gx;

  // Adaptive Kalman
  float accelMag = sqrt(ax*ax + ay*ay + az*az);
  smoothMag = 0.9f * smoothMag + 0.1f * accelMag;
  float deviation = abs(smoothMag - 9.8f);

  if      (deviation < 0.3f) { R_measure = 0.02; Q_angle = 0.001; }
  else if (deviation < 1.0f) { R_measure = 0.08; Q_angle = 0.003; }
  else if (deviation < 3.0f) { R_measure = 0.2;  Q_angle = 0.01;  }
  else                        { R_measure = 0.5;  Q_angle = 0.03;  }

  double pitch = Kalman_filter(accelPitch, gyroRateY, angle,  bias,  P);
  double roll  = Kalman_filter(accelRoll,  gyroRateX, angleR, biasR, PR);

  if (now - lastPrintTime >= PRINT_INTERVAL) {

    if (!isCalibrated) { lastPrintTime = now; return; }

    uint32_t ts = baseEpoch + (millis() - baseMillis) / 1000;

    // 20-byte BLE packet: ts(4)+pitch(2)+roll(2)+ax(2)+ay(2)+az(2)+gx(2)+gy(2)+gz(2)
    int16_t iPitch = (int16_t)(pitch * 100);
    int16_t iRoll  = (int16_t)(roll  * 100);
    int16_t iAx    = (int16_t)(ax    * 100);
    int16_t iAy    = (int16_t)(ay    * 100);
    int16_t iAz    = (int16_t)(az    * 100);
    int16_t iGx    = (int16_t)(gx    * 10);  // already deg/s
    int16_t iGy    = (int16_t)(gy    * 10);
    int16_t iGz    = (int16_t)(gz    * 10);

    uint8_t buf[20];
    memcpy(buf,      &ts,     4);
    memcpy(buf +  4, &iPitch, 2);
    memcpy(buf +  6, &iRoll,  2);
    memcpy(buf +  8, &iAx,    2);
    memcpy(buf + 10, &iAy,    2);
    memcpy(buf + 12, &iAz,    2);
    memcpy(buf + 14, &iGx,    2);
    memcpy(buf + 16, &iGy,    2);
    memcpy(buf + 18, &iGz,    2);

    if (deviceConnected) {
      notifyChar->setValue(buf, 20);
      notifyChar->notify();
    }

    // Serial CSV debug
    Serial.print(ts);        Serial.print(",");
    Serial.print(pitch, 2);  Serial.print(",");
    Serial.print(roll,  2);  Serial.print(",");
    Serial.print(ax, 3);     Serial.print(",");
    Serial.print(ay, 3);     Serial.print(",");
    Serial.print(az, 3);     Serial.print(",");
    Serial.print(gx, 2);     Serial.print(",");
    Serial.print(gy, 2);     Serial.print(",");
    Serial.println(gz, 2);

    lastPrintTime = now;
  }
}

/* ═══════════════════ KALMAN FUNCTION ════════════════════════ */
double Kalman_filter(double accelAngle, double gyroRate,
                     double &angle, double &bias, double P[2][2]) {
  double rate = gyroRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  double S  = P[0][0] + R_measure;
  double K0 = P[0][0] / S;
  double K1 = P[1][0] / S;

  double y = accelAngle - angle;
  angle += K0 * y;
  bias  += K1 * y;

  double P00 = P[0][0], P01 = P[0][1];
  P[0][0] -= K0 * P00;
  P[0][1] -= K0 * P01;
  P[1][0] -= K1 * P00;
  P[1][1] -= K1 * P01;

  return angle;
}
