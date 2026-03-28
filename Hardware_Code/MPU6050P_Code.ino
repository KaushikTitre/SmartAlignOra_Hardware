#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* ================= MPU6050 ================= */
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

/* ================= Calibration ================= */
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

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found!");
    while (1);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("MPU6050 Ready");

  calibrateMPU();

  angle = 0;
  bias  = 0;
  P[0][0] = P[0][1] = P[1][0] = P[1][1] = 0;

  lastTime = millis();

  Serial.println("Time(ms), Pitch(deg)");
}

/* ================= LOOP ================= */
void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0;
  lastTime = now;

  double accelPitch = atan2(
    -a.acceleration.x,
    sqrt(a.acceleration.y*a.acceleration.y +
         a.acceleration.z*a.acceleration.z)
  ) * 180.0 / PI;

  accelPitch -= pitchOffset;

  double gyroRate = g.gyro.y * 180.0 / PI;
  double pitch = Kalman_filter(accelPitch, gyroRate);

  // ✅ Print slower (every 200 ms)
  if (now - lastPrintTime >= PRINT_INTERVAL) {
    Serial.print(now);
    Serial.print(",");
    Serial.println(pitch);
    lastPrintTime = now;
  }

  delay(20);  // Keep Kalman stable at ~50 Hz
}
