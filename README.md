# Easy-EQs-prototype

A simple earthquake/vibration detection system using:
- ESP-32S ESP-WROOM-32 Development Board 38P NodeMC
- MPU6050 accelerometer
- SW420 vibration sensor
- Blynk IoT platform for alerts

## Features
- Detects vibrations from both digital and analog sensors
- Triggers a buzzer alarm
- Sends mobile push notifications + email via Blynk.logEvent
- WiFi connectivity with auto-reconnect

## Setup
1. Upload the Arduino sketch to ESP32
2. Configure WiFi credentials and Blynk auth token
3. Connect sensors as per code comments

> **Note:** Replace placeholder WiFi credentials with your actual network details.
