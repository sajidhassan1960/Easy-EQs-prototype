#define BLYNK_TEMPLATE_ID "TMPL6xjdRnwvk"
#define BLYNK_TEMPLATE_NAME "Earthquake Detector"
#define BLYNK_AUTH_TOKEN "PolRx9YsCSFQWdSNjL3prMeHbMoIVNey"

#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>


// WiFi Credentials
char ssid[] = "nnn"; // Replace with your WiFi SSID
char pass[] = "12345678"; // Replace with your WiFi password

// Pins
const int SW420_PIN = 4;      // Vibration sensor
const int BUZZER_PIN = 25;    // Buzzer
const int BUZZER_DURATION = 2000; // 2 sec alarm

// MPU6050 Settings
MPU6050 mpu;
const float ACCEL_THRESHOLD = 1.2;  // Adjusted for realistic earthquake detection (~1.2G)

// Variables
bool earthquakeDetected = false;
unsigned long lastTriggerTime = 0;
unsigned long lastNotificationTime = 0;
const unsigned long NOTIFICATION_COOLDOWN = 60000; // 1 min cooldown for notifications
const unsigned long WIFI_TIMEOUT = 15000; // 15 sec timeout for WiFi connection

// Calibration offsets
float accelXOffset = 0, accelYOffset = 0, accelZOffset = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 23); // SDA=21, SCL=23
  mpu.initialize();

  pinMode(SW420_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Connect to WiFi
  connectToWiFi();

  // Connect to Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("Connected to Blynk!");

  // Check MPU6050 connection
  if (!mpu.testConnection()) {
    Serial.println("❌ MPU6050 not found!");
    while (1); // Stop if MPU6050 fails
  }

  // Optional: Calibrate MPU6050 (uncomment to use)
  // calibrateMPU6050();

  Serial.println("✅ Earthquake Detector Ready!");
  Serial.println("🔊 Tap the table to test!");
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startTime < WIFI_TIMEOUT) {
    delay(500);
    Serial.print(".");
    Serial.print(" Status: ");
    Serial.println(WiFi.status()); // Print WiFi status for debugging
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi connection failed! Restarting...");
    ESP.restart(); // Restart ESP32 if connection fails
  }

  Serial.println("\nConnected to WiFi!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  Blynk.run(); // Run Blynk

  // Check WiFi connection and reconnect if lost
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected! Reconnecting...");
    connectToWiFi();
  }

  // Read SW420 (Digital)
  bool sw420Triggered = digitalRead(SW420_PIN) == HIGH;

  // Read MPU6050 (Analog - Checks Acceleration)
  bool mpuTriggered = checkMPU6050();

  // If either sensor triggers...
  if (sw420Triggered || mpuTriggered) {
    if (!earthquakeDetected) {
      earthquakeDetected = true;
      lastTriggerTime = millis();
      triggerAlarm();
      Serial.println("🚨 EARTHQUAKE DETECTED!");

      // Send Blynk event if cooldown period has passed
      if (millis() - lastNotificationTime > NOTIFICATION_COOLDOWN) {
        Blynk.logEvent("earthquake_alert");
        Serial.println("🚨 EARTHQUAKE ALERT SENT!");
        lastNotificationTime = millis();
      }
    }
  } else {
    if (earthquakeDetected && (millis() - lastTriggerTime > 1000)) {
      earthquakeDetected = false; // Reset after 1 sec
    }
  }

  delay(50); // Small delay for stability
}

// Checks MPU6050 for sudden movement
bool checkMPU6050() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  // Convert raw data to G-forces (MPU6050 default sensitivity: 16384 = 1G)
  float accelX = (ax / 16384.0) - accelXOffset;
  float accelY = (ay / 16384.0) - accelYOffset;
  float accelZ = (az / 16384.0) - accelZOffset;

  // Calculate acceleration magnitude
  float accelMagnitude = sqrt(accelX * accelX + accelY * accelY + accelZ * accelZ);

  // Debugging: Print acceleration values
  Serial.print("Accel (G): X=");
  Serial.print(accelX);
  Serial.print(", Y=");
  Serial.print(accelY);
  Serial.print(", Z=");
  Serial.print(accelZ);
  Serial.print(", Magnitude=");
  Serial.println(accelMagnitude);

  if (accelMagnitude > ACCEL_THRESHOLD) {
    return true;  // Movement detected!
  }
  return false;
}

// Optional: Calibrate MPU6050 to remove offsets
void calibrateMPU6050() {
  Serial.println("Calibrating MPU6050... Keep sensor still.");
  int32_t axSum = 0, aySum = 0, azSum = 0;
  const int samples = 100;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    axSum += ax;
    aySum += ay;
    azSum += az;
    delay(10);
  }

  // Calculate offsets (Z should be ~1G due to gravity, others ~0)
  accelXOffset = (axSum / samples) / 16384.0;
  accelYOffset = (aySum / samples) / 16384.0;
  accelZOffset = ((azSum / samples) / 16384.0) - 1.0; // Subtract 1G for gravity

  Serial.println("Calibration complete!");
  Serial.print("Offsets: X=");
  Serial.print(accelXOffset);
  Serial.print(", Y=");
  Serial.print(accelYOffset);
  Serial.print(", Z=");
  Serial.println(accelZOffset);
}

// Sounds the buzzer for 2 seconds
void triggerAlarm() {
  tone(BUZZER_PIN, 1000); // 1000Hz tone
  delay(BUZZER_DURATION);
  noTone(BUZZER_PIN); // Turn off
}