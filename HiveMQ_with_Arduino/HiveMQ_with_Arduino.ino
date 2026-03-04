/*
  HiveMQ_with_Arduino
  Arduino UNO R4 WiFi + Modulino Thermo + Modulino Buzzer
  Connects to HiveMQ Cloud MQTT broker over TLS (port 8883).
  Publishes sensor data (temperature, humidity, alarm status) every 60 seconds.
  Subscribes to alarm topic and triggers buzzer when temperature > 35 C.
*/

#include <WiFiS3.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <Modulino.h>

#include "config.h"
#include "wifi_utils.h"
#include "xymd02_utils.h"
#include "modulino_utils.h"
#include "mqtt_utils.h"

// ── Objects ─────────────────────────────────────────────────────────
WiFiSSLClient  wifiClient;
MqttClient     mqttClient(wifiClient);
ModulinoThermo thermo;
ModulinoBuzzer buzzer;

// ── Topics (built with MAC address in setup) ────────────────────────
String topicPublish;    // /<mac>/sensorData
String topicSubscribe;  // /<mac>/alarm/status

// ── State ───────────────────────────────────────────────────────────
unsigned long previousMillis = 0;
bool allOk           = false;
bool modulinoOk      = false;  // Modulino Thermo + Buzzer available
bool xymd02Ok        = false;  // XY-MD02 RS-485 sensor available
bool alarmActive     = false;
bool alarmTestActive = false;
unsigned long sosStepStart = 0;
int sosStepIndex = 0;

// ═════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < TIMEOUT) {}

  Serial.println("=== HiveMQ MQTT Sensor Station ===");
  Serial.println();

  // 1. Connect WiFi
  if (!connectWiFi()) {
    Serial.println("Cannot proceed without WiFi.");
    return;
  }

  // 2. Build MQTT topics using MAC address
  String mac = getMacAddress();
  mac.replace(":", "");
  topicPublish   = "/" + mac + "/sensorData";
  topicSubscribe = "/" + mac + "/alarm/status";

  // 3. Connect MQTT
  if (!connectMQTT()) {
    Serial.println("Cannot proceed without MQTT.");
    return;
  }

  // 4. Initialize sensors (continue if at least one succeeds)
  modulinoOk = initModulino();
  if (!modulinoOk) Serial.println("Warning: Modulino unavailable, continuing...");

  xymd02Ok = initXYMD02();
  if (!xymd02Ok) Serial.println("Warning: XY-MD02 unavailable, continuing...");

  if (!modulinoOk && !xymd02Ok) {
    Serial.println("Cannot proceed: no sensors available.");
    return;
  }

  Serial.println();
  Serial.println("All services started successfully.");
  Serial.println("Publishing every 60 seconds...");
  Serial.println();

  // Set MQTT message handler
  mqttClient.onMessage(onMqttMessage);

  allOk = true;
}

void loop() {
  if (!allOk) return;

  // Keep MQTT connection alive and process incoming messages
  mqttClient.poll();

  // Reconnect if disconnected
  if (!mqttClient.connected()) {
    Serial.println("MQTT disconnected, reconnecting...");
    connectMQTT();
  }

  // SOS pattern state machine (buzzer requires Modulino)
  if (modulinoOk && alarmTestActive && millis() - sosStepStart >= SOS_PATTERN[sosStepIndex]) {
    sosStepIndex++;
    if (sosStepIndex >= SOS_PATTERN_LEN) {
      // SOS complete
      alarmTestActive = false;
      buzzer.noTone();
      Serial.println("Alarm TEST: SOS signal complete");
    } else {
      sosStepStart = millis();
      if (sosStepIndex % 2 == 0) {
        // Even index = tone step
        buzzer.tone(1000, SOS_PATTERN[sosStepIndex]);
      } else {
        // Odd index = gap step (silence)
        buzzer.noTone();
      }
    }
  }

  // Publish on interval (non-blocking)
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= PUBLISH_INTERVAL) {
    previousMillis = currentMillis;
    publishSensorData();
  }
}
