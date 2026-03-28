// ===== PWM Settings =====
const int motorPin = 26;      // Change if needed
const int pwmChannel = 0;
const int pwmFreq = 1000;     // 1 kHz (better for coin motor)
const int pwmResolution = 8;  // 8-bit (0–255)

void setup() {
  Serial.begin(115200);

  // New ESP32 Core 3.x PWM setup
  ledcAttach(motorPin, pwmFreq, pwmResolution);

  Serial.println("PWM Vibration Test Started");
}

void loop() {

  // Increase vibration
  for (int duty = 0; duty <= 255; duty += 5) {
    ledcWrite(motorPin, duty);
    Serial.print("Increasing: ");
    Serial.println(duty);
    delay(100);
  }

  delay(1000);

  // Decrease vibration
  for (int duty = 255; duty >= 0; duty -= 5) {
    ledcWrite(motorPin, duty);
    Serial.print("Decreasing: ");
    Serial.println(duty);
    delay(100);
  }

  delay(1000);
}