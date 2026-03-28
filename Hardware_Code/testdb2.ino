#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <time.h>
#include <Wire.h>

/* ================= WiFi ================= */
const char* ssid = "Tanay";
const char* password = "tan@y123";

const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 19800;
const int daylightOffset_sec = 0;

/* ================= MPU6050 ================= */
Adafruit_MPU6050 mpu;

/* ================= Kalman Variables ================= */
double Q_angle = 0.001;
double Q_bias  = 0.003;
double R_measure = 0.03;

// Pitch Kalman
double angleP = 0, biasP = 0;
double PP[2][2] = {{0,0},{0,0}};

// Roll Kalman
double angleR = 0, biasR = 0;
double PR[2][2] = {{0,0},{0,0}};

double dt;
unsigned long lastTime;
unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 20;

/* ================= Calibration ================= */
double pitchOffset = 0;
double rollOffset  = 0;

/* ================= Kalman Function ================= */
double Kalman_filter(double accelAngle, double gyroRate,
                     double &angle, double &bias, double P[2][2]) {
  double rate = gyroRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
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

/* ================= Calibration ================= */
void calibrateMPU() {
  Serial.println("Calibrating... Keep device steady");

  double sumPitch = 0, sumRoll = 0;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sumPitch += atan2(
      -a.acceleration.x,
      sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)
    ) * 180.0 / PI;

    sumRoll += atan2(
      a.acceleration.y,
      sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.z*a.acceleration.z)
    ) * 180.0 / PI;

    delay(5);
  }

  pitchOffset = sumPitch / samples;
  rollOffset  = sumRoll  / samples;

  Serial.print("Pitch Offset: "); Serial.println(pitchOffset);
  Serial.print("Roll Offset:  "); Serial.println(rollOffset);
}

/* ================= Unix Timestamp ================= */
unsigned long getUnixTimestamp() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return 0;
  return (unsigned long)mktime(&timeinfo);
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  Wire.begin();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nWiFi connected");

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  // Wait for NTP sync
  Serial.print("Syncing NTP");
  struct tm timeinfo;
  while (!getLocalTime(&timeinfo)) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("\nNTP synced");

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  Serial.println("MPU6050 Ready");

  calibrateMPU();

  angleP = angleR = biasP = biasR = 0;
  PP[0][0]=PP[0][1]=PP[1][0]=PP[1][1]=0;
  PR[0][0]=PR[0][1]=PR[1][0]=PR[1][1]=0;

  lastTime = millis();

  // CSV Header
  Serial.println("timestamp_ms,pitch,roll,ax,ay,az,acc_mag,gx,gy,gz");
}

/* ================= LOOP ================= */
void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

 float mag = sqrt(
    a.acceleration.x * a.acceleration.x +
    a.acceleration.y * a.acceleration.y +
    a.acceleration.z * a.acceleration.z
  );
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  if (dt <= 0 || dt > 0.1) dt = 0.02;
  lastTime = now;

  // Accel angles
  double accelPitch = atan2(
    -a.acceleration.x,
    sqrt(a.acceleration.y*a.acceleration.y + a.acceleration.z*a.acceleration.z)
  ) * 180.0 / PI - pitchOffset;

  double accelRoll = atan2(
    a.acceleration.y,
    sqrt(a.acceleration.x*a.acceleration.x + a.acceleration.z*a.acceleration.z)
  ) * 180.0 / PI - rollOffset;

  // Gyro rates (deg/s)
  double gyroRateY = g.gyro.y * 180.0 / PI;  // for pitch
  double gyroRateX = g.gyro.x * 180.0 / PI;  // for roll

  // Kalman filter
  double pitch = Kalman_filter(accelPitch, gyroRateY, angleP, biasP, PP);
  double roll  = Kalman_filter(accelRoll,  gyroRateX, angleR, biasR, PR);

  if (now - lastPrintTime >= PRINT_INTERVAL) {
    // unsigned long ts = getUnixTimestamp();
    uint64_t ts = esp_timer_get_time() / 1000;   // milliseconds

  Serial.print(ts);                          Serial.print(",");
Serial.print(pitch, 2);                    Serial.print(",");
Serial.print(roll, 2);                     Serial.print(",");
Serial.print(a.acceleration.x, 4);         Serial.print(",");
Serial.print(a.acceleration.y, 4);         Serial.print(",");
Serial.print(a.acceleration.z, 4);         Serial.print(",");

Serial.print(mag, 4);                      Serial.print(",");   // ✅ ADD HERE

Serial.print(g.gyro.x * 180.0/PI, 4);      Serial.print(",");
Serial.print(g.gyro.y * 180.0/PI, 4);      Serial.print(",");
Serial.println(g.gyro.z * 180.0/PI, 4);

    lastPrintTime = now;
  }
}