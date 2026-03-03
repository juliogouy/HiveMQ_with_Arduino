#ifndef MODULINO_UTILS_H
#define MODULINO_UTILS_H

#include <Modulino.h>
#include <ArduinoJson.h>
#include <WiFiS3.h>
#include "config.h"

// ── External references (defined in main .ino) ─────────────────────
extern MqttClient      mqttClient;
extern ModulinoThermo  thermo;
extern ModulinoBuzzer  buzzer;
extern String topicPublish;
extern bool alarmActive;
extern bool alarmTestActive;

// ── SOS alarm test pattern (non-blocking) ───────────────────────────
// SOS pattern: . . . - - - . . .
// Short tone = 250ms, Long tone = 750ms, Gap between tones = 500ms
// Pattern: tone_ms, gap_ms, tone_ms, gap_ms, ...  (ends with last tone, no trailing gap)
const unsigned int SOS_PATTERN[] = {
   250,  500,  // S: dot
   250,  500,  //    dot
   250,  500,  //    dot + gap before O
   750,  500,  // O: dash
   750,  500,  //    dash
   750,  500,  //    dash + gap before S
   250,  500,  // S: dot
   250,  500,  //    dot
   250          //    dot (last, no trailing gap)
};
const int SOS_PATTERN_LEN = sizeof(SOS_PATTERN) / sizeof(SOS_PATTERN[0]);

// ── Initialize Modulino devices ─────────────────────────────────────
bool initModulino() {
  Modulino.begin();
  Serial.println("Modulino I2C: OK");

  unsigned long start = millis();
  bool thermoOk = false;
  while (millis() - start < TIMEOUT) {
    if (thermo.begin()) {
      thermoOk = true;
      break;
    }
  }
  if (thermoOk) {
    Serial.println("Modulino Thermo: OK");
  } else {
    Serial.println("Modulino Thermo: FAILED");
    return false;
  }

  start = millis();
  bool buzzerOk = false;
  while (millis() - start < TIMEOUT) {
    if (buzzer.begin()) {
      buzzerOk = true;
      break;
    }
  }
  if (buzzerOk) {
    Serial.println("Modulino Buzzer: OK");
  } else {
    Serial.println("Modulino Buzzer: FAILED");
    return false;
  }

  return true;
}

// ── Publish sensor data as JSON ─────────────────────────────────────
void publishSensorData() {
  float temperature = thermo.getTemperature();
  float humidity    = thermo.getHumidity();

  // Alarm logic (skip if alarm test is running)
  if (!alarmTestActive) {
    if (temperature > ALARM_THRESHOLD) {
      alarmActive = true;
      buzzer.tone(1000, 500);  // 1kHz tone for 500ms
    } else {
      alarmActive = false;
      buzzer.noTone();
    }
  }

  const char* alarmStatus = alarmActive ? "alarmOn" : "alarmOff";

  // Build JSON
  JsonDocument doc;
  JsonObject sensorData = doc["sensorData"].to<JsonObject>();
  JsonArray modulinoThermo = sensorData["ModulinoThermo"].to<JsonArray>();

  JsonObject ts = modulinoThermo.add<JsonObject>();
  ts["timestamp"] = WiFi.getTime();  // Epoch time (UTC) via NTP

  JsonObject temp = modulinoThermo.add<JsonObject>();
  temp["temperature"] = serialized(String(temperature, 1));

  JsonObject hum = modulinoThermo.add<JsonObject>();
  hum["humidity"] = serialized(String(humidity, 1));

  JsonObject alarm = modulinoThermo.add<JsonObject>();
  alarm["alarmStatus"] = alarmStatus;

  // Publish
  mqttClient.beginMessage(topicPublish);
  serializeJson(doc, mqttClient);
  mqttClient.endMessage();

  // Serial output
  Serial.print("Published to ");
  Serial.print(topicPublish);
  Serial.print(": ");
  serializeJson(doc, Serial);
  Serial.println();
}

#endif
