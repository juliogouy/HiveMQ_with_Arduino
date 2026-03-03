#ifndef MQTT_UTILS_H
#define MQTT_UTILS_H

#include <ArduinoMqttClient.h>
#include "config.h"

// ── External references (defined in main .ino) ─────────────────────
extern MqttClient    mqttClient;
extern ModulinoBuzzer buzzer;
extern String topicPublish;
extern String topicSubscribe;
extern bool alarmActive;
extern bool alarmTestActive;
extern unsigned long sosStepStart;
extern int sosStepIndex;
extern const unsigned int SOS_PATTERN[];

// ── Connect to MQTT broker ──────────────────────────────────────────
bool connectMQTT() {
  Serial.print("Connecting to MQTT broker: ");
  Serial.println(MQTT_BROKER);

  mqttClient.setUsernamePassword(MQTT_USER, MQTT_PASS);

  int attempts = 0;
  while (!mqttClient.connect(MQTT_BROKER, MQTT_PORT) && attempts < 5) {
    attempts++;
    Serial.print("MQTT connection failed, error code: ");
    Serial.println(mqttClient.connectError());
    Serial.println("Retrying in 5s...");
    delay(5000);
  }

  if (!mqttClient.connected()) {
    Serial.println("MQTT Broker: FAILED");
    return false;
  }

  Serial.println("MQTT Broker: OK");

  // Subscribe to alarm topic and test topic
  mqttClient.subscribe(topicSubscribe);
  mqttClient.subscribe(TOPIC_ALARM_TEST);
  Serial.print("Subscribed to: ");
  Serial.println(topicSubscribe);
  Serial.print("Subscribed to: ");
  Serial.println(TOPIC_ALARM_TEST);
  Serial.print("Publishing to: ");
  Serial.println(topicPublish);

  return true;
}

// ── Handle incoming MQTT messages ───────────────────────────────────
void onMqttMessage(int messageSize) {
  String topic = mqttClient.messageTopic();
  String payload;
  while (mqttClient.available()) {
    payload += (char)mqttClient.read();
  }

  Serial.print("Received [");
  Serial.print(topic);
  Serial.print("]: ");
  Serial.println(payload);

  // Handle alarm test topic -- start SOS pattern
  if (topic == TOPIC_ALARM_TEST && payload == "teston") {
    alarmTestActive = true;
    sosStepIndex = 0;
    sosStepStart = millis();
    buzzer.tone(1000, SOS_PATTERN[0]);  // start first dot
    Serial.println("Alarm TEST: SOS signal started");
    return;
  }

  // Handle alarm status topic
  if (payload == "on") {
    alarmActive = true;
    buzzer.tone(1000, 500);
    Serial.println("Alarm triggered via MQTT");
  } else if (payload == "off") {
    alarmActive = false;
    buzzer.noTone();
    Serial.println("Alarm cleared via MQTT");
  }
}

#endif
