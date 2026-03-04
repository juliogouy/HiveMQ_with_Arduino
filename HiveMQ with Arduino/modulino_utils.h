#ifndef MODULINO_UTILS_H
#define MODULINO_UTILS_H

#include <Modulino.h>
#include <ArduinoJson.h>
#include <WiFiS3.h>
#include "config.h"
#include "xymd02_utils.h"

// ── External references (defined in main .ino) ─────────────────────
extern MqttClient      mqttClient;
extern ModulinoThermo  thermo;
extern ModulinoBuzzer  buzzer;
extern String topicPublish;
extern bool alarmActive;
extern bool alarmTestActive;
extern bool modulinoOk;
extern bool xymd02Ok;

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
  // Read whichever sensors are available
  float modulinoTemp, modulinoHum;
  bool modulinoRead = false;
  if (modulinoOk) {
    modulinoTemp  = thermo.getTemperature();
    modulinoHum   = thermo.getHumidity();
    modulinoRead  = true;
  }

  float xyTemp, xyHum;
  bool xyRead = false;
  if (xymd02Ok) {
    xyRead = readXYMD02(xyTemp, xyHum);
  }

  // Alarm: trigger if any available sensor exceeds threshold
  bool tempAlarm = false;
  if (modulinoRead)        tempAlarm |= (modulinoTemp > ALARM_THRESHOLD);
  if (xymd02Ok && xyRead)  tempAlarm |= (xyTemp > ALARM_THRESHOLD);

  if (!alarmTestActive) {
    alarmActive = tempAlarm;
    if (modulinoOk) {
      alarmActive ? buzzer.tone(1000, 500) : buzzer.noTone();
    }
  }

  const char* alarmStatus = alarmActive ? "alarmOn" : "alarmOff";

  // Build JSON
  JsonDocument doc;
  JsonObject sensorData = doc["sensorData"].to<JsonObject>();
  sensorData["timestamp"]   = WiFi.getTime();  // Epoch time (UTC) via NTP
  sensorData["alarmStatus"] = alarmStatus;

  // ModulinoThermo
  JsonArray modulinoArr = sensorData["ModulinoThermo"].to<JsonArray>();
  if (modulinoRead) {
    JsonObject t = modulinoArr.add<JsonObject>();
    t["temperature"] = serialized(String(modulinoTemp, 1));
    JsonObject h = modulinoArr.add<JsonObject>();
    h["humidity"] = serialized(String(modulinoHum, 1));
  } else {
    JsonObject e = modulinoArr.add<JsonObject>();
    e["error"] = "not available";
  }

  // XY-MD02
  JsonArray xyArr = sensorData["XYMD02"].to<JsonArray>();
  if (!xymd02Ok) {
    JsonObject e = xyArr.add<JsonObject>();
    e["error"] = "not available";
  } else if (!xyRead) {
    JsonObject e = xyArr.add<JsonObject>();
    e["error"] = "read failed";
  } else {
    JsonObject t = xyArr.add<JsonObject>();
    t["temperature"] = serialized(String(xyTemp, 1));
    JsonObject h = xyArr.add<JsonObject>();
    h["humidity"] = serialized(String(xyHum, 1));
  }

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
