#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>

bool needCalibration = false;
bool isCalibrated = false;  

volatile bool deviceConnected     = false;
volatile bool previouslyConnected = false;  

//Add PWM variables
#define VIBRATION_PIN 26 // vibration motor connected to PIN 26
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8

int vibrationIntensity = 127;   // default 50% (PWM range 0–255)

// Vibration Pattern
int currentPattern = 0;  // 0 = no pattern
bool patternRunning = false;
unsigned long patternTimer = 0;
int patternStep = 0;

//************************BLE*******************************
/* UUIDs */
#define SERVICE_UUID        "a32be81d-570e-4ad9-bf2a-64fdfe3db515"
#define WRITE_CHAR_UUID    "c6b0278a-f9b5-4306-8eee-5d74d746bcc3"
#define READ_CHAR_UUID     "f53d0832-1177-479b-8046-1b76534390cb"
#define NOTIFY_CHAR_UUID   "b3af0550-1ba9-4efb-aac7-14287a527e06"

/* BLE Objects */
BLEServer *pServer;
BLECharacteristic *writeChar;
BLECharacteristic *readChar;
BLECharacteristic *notifyChar;

/* State variables */
bool timeSynced = false;
uint32_t currentTime;

/* Time sync variables */
uint32_t baseEpoch = 0;
uint32_t baseMillis = 0;

/* Handshake timer */
unsigned long connectTime = 0;
const unsigned long HANDSHAKE_TIMEOUT = 10000; // 10 seconds

/* ================= WRITE CALLBACK ================= */
class TimeWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {

    String value = pChar->getValue();
    Serial.print("BLE Received Command: ");
Serial.println(value);

    if (value.length() == 0) return;

    // ---------- HANDSHAKE ----------
    if (!timeSynced) {

      uint32_t receivedTime = value.toInt();

      if (receivedTime < 1600000000) {
        Serial.println("Invalid timestamp received");
        return;
      }

      baseEpoch = receivedTime;
      baseMillis = millis();
      timeSynced = true;

      Serial.print("Timestamp received: ");
      Serial.println(baseEpoch);
      Serial.println("Handshake successful!");

      return;
    }

    // ---------- CALIBRATION ---------- 
if (value == "C1") {
  needCalibration = true;
  Serial.println("Calibration triggered by app (C1)");
  return;
}

// ------- VIBRATION PATTERNS ----------
if (value.startsWith("S")) {
  int patNum = value.substring(1).toInt();

  if (patNum >= 1 && patNum <= 6) {
    ledcWrite(VIBRATION_PIN, 0);  // stop current before switching

    currentPattern = patNum;
    patternRunning = true;
    patternStep = 0;
    patternTimer = millis();

    Serial.print("Pattern started: S");
    Serial.println(patNum);
  } else {
    Serial.println("Invalid pattern number");
  }
  return;
}

// ------- MOTOR INTENSITY CONTROL ----------
if (value.startsWith("V")) {

  int percent = value.substring(1).toInt();
  percent = constrain(percent, 0, 100);

  vibrationIntensity = map(percent, 0, 100, 0, 255);

  Serial.print("Vibration intensity set to ");
  Serial.print(percent);
  Serial.println("%");

  // If v0 → stop motor completely
  if (percent == 0) {
    ledcWrite(VIBRATION_PIN, 0);
    Serial.println("Motor stopped (v0 received)");
    return;
  }

  // ✅ Update intensity live if pattern is running
  if (patternRunning) {
    Serial.println("Intensity updated, applies on next pattern cycle");
  }

  return;
}


if (value == "0") {
  // Stop pattern too
  patternRunning = false;
  patternStep = 0;
  currentPattern = 0;

  Serial.println("Motor OFF");
  ledcWrite(VIBRATION_PIN, 0);
  return;
}

  }
};

/* ================= SERVER CALLBACKS ================= */
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) {
    deviceConnected = true;
    timeSynced = false;
    connectTime = millis();
    Serial.println("Client connected, waiting for timestamp...");
  }

void onDisconnect(BLEServer* server) override {
  vibrationIntensity = 127;
  patternRunning     = false;   // ✅ add
  patternStep        = 0;       // ✅ add
  currentPattern     = 0;       // ✅ add
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
  BLEDevice::startAdvertising();   // Core 3.x requires this, not advertising->start()
  Serial.println("[BLE] Advertising restarted");
}

//**************************MPU6050***************************
Adafruit_MPU6050 mpu;

/* ================= Kalman Variables ================= */
double Q_angle = 0.001;
double Q_bias  = 0.003;
double R_measure = 0.03;

double angle = 0;
double bias  = 0;
double rate  = 0;

double P[2][2] = {{0, 0}, {0, 0}};
double dt;
unsigned long lastTime;

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 200; // ms

/* ================= Calibration ================= */
double pitchOffset = 0;

/* ===========MPU6050  Calibration ================= */
void calibrateMPU() {
  Serial.println("Calibrating... Keep device steady");

  double sum = 0;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    double pitch = atan2(
      -a.acceleration.x,
      sqrt(a.acceleration.y*a.acceleration.y +
           a.acceleration.z*a.acceleration.z)
    ) * 180.0 / PI;

    sum += pitch;
    delay(5);
  }

  pitchOffset = sum / samples;
  Serial.print("Pitch Offset: ");
  Serial.println(pitchOffset);
}

/* ================= SETUP ================= */
void setup() {
 Serial.begin(115200);
     Wire.begin();

// PWM configuration for vibration motor
ledcAttach(VIBRATION_PIN, PWM_FREQ, PWM_RESOLUTION);
ledcWrite(VIBRATION_PIN, 0);

     // MPU6050 Initialization
   Serial.println("Initializing MPU6050...");
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050!");
    while (1);
  }
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 Ready!");
  
  lastTime = millis();

  //*****************BLE*******************
 BLEDevice::init("SmartAlignora");
 BLEDevice::setPower(ESP_PWR_LVL_P9);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *service = pServer->createService(SERVICE_UUID);

  // WRITE characteristic (timestamp)
  writeChar = service->createCharacteristic(
                WRITE_CHAR_UUID,
                BLECharacteristic::PROPERTY_WRITE
              );
  writeChar->setCallbacks(new TimeWriteCallback());

  // READ characteristic
  readChar = service->createCharacteristic(
               READ_CHAR_UUID,
               BLECharacteristic::PROPERTY_READ
             );
  readChar->setValue("ESP32 Ready");

  // NOTIFY characteristic
  notifyChar = service->createCharacteristic(
                 NOTIFY_CHAR_UUID,
                 BLECharacteristic::PROPERTY_NOTIFY
               );
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
Serial.println("Device Name: SmartAlignora");
Serial.println("Waiting for phone connection...");
Serial.println("================================");
}

/* ================= Vibration Patterns ================= */
void runVibrationPattern() {
  if (!patternRunning) return;

  unsigned long now = millis();

  switch (currentPattern) {

    case 1: // S1 - Short (loops: 200ms ON, 300ms OFF)
      if (patternStep == 0) { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 1 && now - patternTimer >= 200) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 2 && now - patternTimer >= 300) { patternStep = 0; } // ✅ loop
      break;

    case 2: // S2 - Medium (loops: 500ms ON, 300ms OFF)
      if (patternStep == 0) { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 1 && now - patternTimer >= 500) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 2 && now - patternTimer >= 300) { patternStep = 0; } // ✅ loop
      break;

    case 3: // S3 - Long (loops: 1000ms ON, 300ms OFF)
      if (patternStep == 0) { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 1 && now - patternTimer >= 1000) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 2 && now - patternTimer >= 300) { patternStep = 0; } // ✅ loop
      break;

    case 4: // S4 - Tuk Tuk (loops: tuk-tuk pause repeat)
      if (patternStep == 0) { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 1 && now - patternTimer >= 150) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 2 && now - patternTimer >= 100) { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 3 && now - patternTimer >= 150) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 4 && now - patternTimer >= 400) { patternStep = 0; } // ✅ loop
      break;

    case 5: // S5 - Knock Knock (loops: knock-knock pause repeat)
      if (patternStep == 0) { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 1 && now - patternTimer >= 200) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 2 && now - patternTimer >= 300) { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 3 && now - patternTimer >= 200) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 4 && now - patternTimer >= 500) { patternStep = 0; } // ✅ loop
      break;

    case 6: // S6 - Heartbeat (already looping — no change needed)
      if (patternStep == 0) { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 1 && now - patternTimer >= 100) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 2 && now - patternTimer >= 100) { ledcWrite(VIBRATION_PIN, vibrationIntensity); patternTimer = now; patternStep++; }
      else if (patternStep == 3 && now - patternTimer >= 100) { ledcWrite(VIBRATION_PIN, 0); patternTimer = now; patternStep++; }
      else if (patternStep == 4 && now - patternTimer >= 500) { patternStep = 0; } // ✅ already loops
      break;

    default:
      patternRunning = false;
      patternStep = 0;
      ledcWrite(VIBRATION_PIN, 0);
      break;
  }
}

/* ================= LOOP ================= */
void loop() {
    // ---- RUN ONLY AFTER HANDSHAKE ----
if (!deviceConnected && previouslyConnected) {
  previouslyConnected = false;
  restartAdvertising();
  return;
}

if (!deviceConnected) {
  delay(100);
  return;
}
  // ---- HANDSHAKE TIMEOUT ----
  if (!timeSynced) {
    if (millis() - connectTime > HANDSHAKE_TIMEOUT) {
      Serial.println("Handshake failed. Disconnecting client...");
      pServer->disconnect(pServer->getConnId());
      delay(100);
    }
    return;
  }

  // ---- CALIBRATION ----
if (needCalibration) {
  calibrateMPU();
  angle = 0;
  bias = 0;
  P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0;
  lastTime = millis();
  lastPrintTime = millis();
  needCalibration = false;
  isCalibrated = true;  // ✅ add this
  return;
}


// ---- RUN PATTERN ----
runVibrationPattern();

  // ---- SENSOR READ ----
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  if (dt <= 0 || dt > 0.1) dt = 0.02;  //This prevents Kalman from going unstable if there's any delay spike during BLE operations.
  lastTime = now;

  double accelPitch = atan2(
    -a.acceleration.x,
    sqrt(a.acceleration.y*a.acceleration.y +
         a.acceleration.z*a.acceleration.z)
  ) * 180.0 / PI;

  accelPitch -= pitchOffset;

  double gyroRate = g.gyro.y * 180.0 / PI;
  double pitch = Kalman_filter(accelPitch, gyroRate);

if (now - lastPrintTime >= PRINT_INTERVAL) {

  if (!isCalibrated) {  
    lastPrintTime = now;
    return;
  }
currentTime = baseEpoch + (millis() - baseMillis) / 1000;

  char csvBuffer[32];
  snprintf(csvBuffer, sizeof(csvBuffer),
           "%.2f,%lu", pitch, currentTime);

if (deviceConnected) {
  notifyChar->setValue(csvBuffer);
  if (deviceConnected) notifyChar->notify();   
}

  Serial.println(csvBuffer);
  lastPrintTime = now;
}

  delay(20);  // Keep Kalman stable at ~50 Hz
}



/* ================= Kalman Function ================= */
double Kalman_filter(double accelAngle, double gyroRate) {
  rate = gyroRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  double S = P[0][0] + R_measure;
  double K0 = P[0][0] / S;
  double K1 = P[1][0] / S;

  double y = accelAngle - angle;
  angle += K0 * y;
  bias  += K1 * y;

  double P00 = P[0][0];
  double P01 = P[0][1];

  P[0][0] -= K0 * P00;
  P[0][1] -= K0 * P01;
  P[1][0] -= K1 * P00;
  P[1][1] -= K1 * P01;

  return angle;
}