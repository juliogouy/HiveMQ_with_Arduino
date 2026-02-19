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

// ── WiFi credentials ────────────────────────────────────────────────
const char WIFI_SSID[] = "YOUR_WIFI_SSID";
const char WIFI_PASS[] = "YOUR_WIFI_PASSWORD";

// ── MQTT broker settings ────────────────────────────────────────────
const char MQTT_BROKER[] = "YOUR_HIVEMQ_BROKER.s2.eu.hivemq.cloud";
const int  MQTT_PORT     = 8883;
const char MQTT_USER[]   = "YOUR_MQTT_USERNAME";
const char MQTT_PASS[]   = "YOUR_MQTT_PASSWORD";

// ── Timing ──────────────────────────────────────────────────────────
const unsigned long PUBLISH_INTERVAL = 60000;
const unsigned long TIMEOUT          = 5000;
const float ALARM_THRESHOLD          = 35.0;

// ── Objects ─────────────────────────────────────────────────────────
WiFiSSLClient wifiClient;
MqttClient    mqttClient(wifiClient);
ModulinoThermo thermo;
ModulinoBuzzer buzzer;

// ── Topics (built with MAC address in setup) ────────────────────────
String topicPublish;    // /<mac>/sensorData
String topicSubscribe;  // /<mac>/alarm/status
const char TOPIC_ALARM_TEST[] = "/alarmTest";

// ── State ───────────────────────────────────────────────────────────
unsigned long previousMillis = 0;
bool allOk = false;
bool alarmActive = false;

// ── SOS alarm test state (non-blocking) ─────────────────────────────
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
bool alarmTestActive = false;
unsigned long sosStepStart = 0;
int sosStepIndex = 0;

// ── Get WiFi MAC address as string ──────────────────────────────────
String getMacAddress() {
  byte mac[6];
  WiFi.macAddress(mac);
  String macStr;
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 0x10) macStr += "0";
    macStr += String(mac[i], HEX);
    if (i > 0) macStr += ":";
  }
  macStr.toUpperCase();
  return macStr;
}

// ── Connect to WiFi ─────────────────────────────────────────────────
bool connectWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  int status = WL_IDLE_STATUS;
  int attempts = 0;
  while (status != WL_CONNECTED && attempts < 5) {
    status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    if (status != WL_CONNECTED) {
      attempts++;
      Serial.println("WiFi connection failed, retrying in 5s...");
      delay(5000);
    }
  }

  if (status != WL_CONNECTED) {
    Serial.println("WiFi: FAILED");
    return false;
  }

  Serial.println("WiFi: OK");
  Serial.print("Waiting for IP address");
  unsigned long ipStart = millis();
  while (WiFi.localIP() == IPAddress(0, 0, 0, 0) && millis() - ipStart < 10000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC address: ");
  Serial.println(getMacAddress());
  return true;
}

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

  // Handle alarm test topic — start SOS pattern
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

  // 4. Initialize Modulino sensors
  if (!initModulino()) {
    Serial.println("Cannot proceed without sensors.");
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

  // SOS pattern state machine
  if (alarmTestActive && millis() - sosStepStart >= SOS_PATTERN[sosStepIndex]) {
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
