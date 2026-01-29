#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

bool needCalibration = false;

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
bool deviceConnected = false;
bool timeSynced = false;
uint32_t currentTime;

/* Time sync variables */
uint32_t baseEpoch = 0;
uint32_t baseMillis = 0;

/* Handshake timer */
unsigned long connectTime = 0;
const unsigned long HANDSHAKE_TIMEOUT = 5000; // 5 seconds

/* ================= WRITE CALLBACK ================= */
class TimeWriteCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pChar) {

    String value = pChar->getValue();   // Arduino String

    if (value.length() == 0) return;

    uint32_t receivedTime = value.toInt();

    // Sanity check (>= year 2020)
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
      // Perform Initial Calibration
   needCalibration = true;
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

  void onDisconnect(BLEServer* server) {
    deviceConnected = false;
    timeSynced = false;
    Serial.println("Client disconnected");
    server->getAdvertising()->start();
  }
};

//**************************MPU6050***************************
// Kalman Filter Variables **********************************
double Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;
double angle = 0, bias = 0, rate = 0;
double P[2][2] = {{0, 0}, {0, 0}};
double dt;
unsigned long lastTime;

Adafruit_MPU6050 mpu;
double pitchoffset = 0;

// MPU6050 Calibration***************************************
void calibrateMPU() {
  Serial.println("Calibrating MPU6050... Please keep the device steady.");

  double pitchSum = 0;
  int numSamples = 500;

  for (int i = 0; i < numSamples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    double accelPitch = atan2(
      -a.acceleration.x,
      sqrt(a.acceleration.y * a.acceleration.y +
           a.acceleration.z * a.acceleration.z)
    ) * 180.0 / PI;

    pitchSum += accelPitch;
    delay(5);
  }

  pitchoffset = pitchSum / numSamples;

  Serial.print("Calibration Complete! Pitch Offset: ");
  Serial.println(pitchoffset);
}

/* ================= SETUP ================= */
void setup() {
 Serial.begin(115200);
   
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
   BLEDevice::init("ESP32_BasicBLE");

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
  advertising->start();

  Serial.println("BLE Advertising Started...");
}

/* ================= LOOP ================= */
void loop() {
    // ---- RUN ONLY AFTER HANDSHAKE ----
  if (!deviceConnected || !timeSynced) return;

  // ---- HANDSHAKE TIMEOUT ----
  if (deviceConnected && !timeSynced) {
    if (millis() - connectTime > HANDSHAKE_TIMEOUT) {
      Serial.println("Handshake failed. Disconnecting client...");
      pServer->disconnect(0);
      delay(100);
    }
    return;
  }
  else if(deviceConnected && timeSynced){

  // ---- CALIBRATION ----
  if (needCalibration) {
    calibrateMPU();

    // Reset Kalman filter state
    angle = 0;
    bias = 0;
    P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0;

    lastTime = millis();   // reset dt reference
    needCalibration = false;
    return;
  }

  // ---- SENSOR READ ----
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  double accelPitch = atan2(
    -a.acceleration.x,
    sqrt(a.acceleration.y * a.acceleration.y +
         a.acceleration.z * a.acceleration.z)
  ) * 180.0 / PI;

  double pitch = Kalman_filter(accelPitch, g.gyro.y);
  pitch -= pitchoffset;

  uint32_t currentTime =
    baseEpoch + (millis() - baseMillis) / 1000;

  char csvBuffer[32];
  snprintf(csvBuffer, sizeof(csvBuffer),
           "%.2f,%lu", pitch, currentTime);

  notifyChar->setValue(csvBuffer);
  notifyChar->notify();

  Serial.println(csvBuffer);
  delay(200); // 5 Hz
  }
}



// Kalman Filter Function************************
double Kalman_filter(double accelAngle, double gyroRate) {
    rate = gyroRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    double S = P[0][0] + R_measure;
    double K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    double y = accelAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    double P00_temp = P[0][0];
    double P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}