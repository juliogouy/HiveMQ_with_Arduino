# HiveMQ MQTT Sensor Station

Arduino UNO R4 WiFi project that reads temperature and humidity from two sensors and publishes the data every 60 seconds to a HiveMQ Cloud MQTT broker over TLS. Subscribes to alarm topics to trigger a buzzer when temperature exceeds 35 °C.

---

## Features

- Publishes sensor data (temperature, humidity, alarm status) every 60 seconds via MQTT over TLS (port 8883)
- Two independent sensors: Modulino Thermo (I2C) and XY-MD02 (RS-485 / Modbus RTU via MAX485)
- Alarm triggers if **either** sensor exceeds 35 °C; falls back to whichever sensor is available if the other fails to initialize
- Subscribes to alarm topics to trigger/clear the buzzer remotely via MQTT
- Alarm test: sends SOS pattern on buzzer (`. . . — — — . . .`) when `/alarmTest` receives `"teston"`
- MQTT topics are built dynamically using the board's WiFi MAC address
- Resilient startup: proceeds with one sensor if the other fails to initialize

---

## Hardware

| Component             | Role                                          | Interface          |
| --------------------- | --------------------------------------------- | ------------------ |
| Arduino UNO R4 WiFi   | Main controller + WiFi + MQTT                 | —                  |
| Modulino Thermo       | Temperature + humidity sensor (SHT40)         | I2C (Modulino bus) |
| Modulino Buzzer       | Alarm output (tone + SOS pattern)             | I2C (Modulino bus) |
| XY-MD02               | Industrial temperature + humidity sensor (SHT20) | RS-485 Modbus RTU  |
| MAX485 TTL adapter    | RS-485 ↔ UART level converter                 | UART (Serial1)     |

### Wiring — MAX485 + XY-MD02

```
Arduino UNO R4 WiFi     MAX485 Module       XY-MD02
────────────────────    ─────────────       ────────
5V              ──────► VCC
GND             ──────► GND
D1 (TX1/Serial1)──────► DI
D0 (RX1/Serial1)◄──────  RO
D2              ──────► DE + RE (tied together)
                          A  ◄────────────── A+
                          B  ◄────────────── B-
                                             VCC ◄── 5V–30V supply
                                             GND ◄── GND
```

> **Note:** The Modulino Thermo and Buzzer connect to the Arduino via the Modulino I2C bus (dedicated connector). No extra wiring needed beyond the standard Modulino chain.

---

## Project Structure

```
HiveMQ with Arduino/
├── HiveMQ_with_Arduino.ino   Main sketch: setup, loop, state variables
├── config.h                  Credentials and constants (NOT committed — see below)
├── wifi_utils.h              WiFi connect + MAC address helpers
├── mqtt_utils.h              MQTT connect + incoming message handler
├── modulino_utils.h          Modulino init, alarm logic, publishSensorData()
├── xymd02_utils.h            XY-MD02 Modbus RTU driver (CRC16, init, read)
├── MQTT_HiveMQ.md            This file
└── XY-MD02_EN_datasheet.pdf  XY-MD02 sensor datasheet
```

### config.h (not committed — create locally)

```cpp
// ── WiFi credentials ────────────────────────────────────────────────
const char WIFI_SSID[] = "YOUR_WIFI_SSID";
const char WIFI_PASS[] = "YOUR_WIFI_PASSWORD";

// ── MQTT broker settings ─────────────────────────────────────────────
const char MQTT_BROKER[] = "YOUR_HIVEMQ_BROKER.s2.eu.hivemq.cloud";
const int  MQTT_PORT     = 8883;
const char MQTT_USER[]   = "YOUR_MQTT_USERNAME";
const char MQTT_PASS[]   = "YOUR_MQTT_PASSWORD";

// ── Timing ───────────────────────────────────────────────────────────
const unsigned long PUBLISH_INTERVAL = 60000;  // ms
const unsigned long TIMEOUT          = 5000;   // ms
const float         ALARM_THRESHOLD  = 35.0;   // °C

// ── XY-MD02 RS-485 / MAX485 settings ─────────────────────────────────
const int  RS485_DE_RE_PIN  = 2;     // MAX485 DE+RE direction control pin
const int  XYMD02_ADDRESS   = 0x01;  // Modbus slave address (default)
const long RS485_BAUD       = 9600;  // 8N1, no parity

// ── Topics ───────────────────────────────────────────────────────────
const char TOPIC_ALARM_TEST[] = "/alarmTest";
```

> **Security:** `config.h` is listed in `.gitignore` and must never be committed. It contains real credentials.

---

## Dependencies

Install via Arduino Library Manager or `arduino-cli lib install`:

| Library           | Purpose                          |
| ----------------- | -------------------------------- |
| `WiFiS3`          | WiFi + NTP (built into R4 core)  |
| `ArduinoMqttClient` | MQTT client over TLS           |
| `ArduinoJson`     | JSON serialization               |
| `Modulino`        | Modulino Thermo + Buzzer drivers |

Board core: `arduino:renesas_uno` (Arduino UNO R4 WiFi)

---

## MQTT Topics

Topics are built at runtime using the board's WiFi MAC address (no colons, uppercase):

| Direction | Topic                        | Payload                        |
| --------- | ---------------------------- | ------------------------------ |
| Publish   | `/<MAC>/sensorData`          | JSON (see below)               |
| Subscribe | `/<MAC>/alarm/status`        | `"on"` / `"off"`               |
| Subscribe | `/alarmTest`                 | `"teston"` triggers SOS buzzer |

---

## JSON Payload Format

Published every 60 seconds to `/<MAC>/sensorData`:

```json
{
  "sensorData": {
    "timestamp": 1741046400,
    "alarmStatus": "alarmOff",
    "ModulinoThermo": [
      {"temperature": "25.0"},
      {"humidity": "60.0"}
    ],
    "XYMD02": [
      {"temperature": "24.8"},
      {"humidity": "58.3"}
    ]
  }
}
```

- `timestamp` — Epoch time (UTC) via NTP, shared across both sensors
- `alarmStatus` — `"alarmOn"` if any available sensor exceeds 35 °C, otherwise `"alarmOff"`
- If a sensor failed to initialize: `{"error": "not available"}`
- If a sensor initialized but a read failed: `{"error": "read failed"}`

---

## Alarm Logic

| Condition                          | Behaviour                                              |
| ---------------------------------- | ------------------------------------------------------ |
| Both sensors OK, either > 35 °C    | `alarmOn`, buzzer sounds 1 kHz for 500 ms              |
| One sensor unavailable             | Alarm based solely on the working sensor               |
| Both sensors unavailable           | Startup aborted, nothing published                     |
| MQTT `/<MAC>/alarm/status` = `on`  | Force alarm on via remote command                      |
| MQTT `/<MAC>/alarm/status` = `off` | Force alarm off via remote command                     |
| MQTT `/alarmTest` = `teston`       | SOS pattern (`. . . — — — . . .`) on buzzer (non-blocking) |

> SOS pattern: 250 ms short tone, 750 ms long tone, 500 ms gaps between tones.

---

## Build and Upload

Requires [Arduino CLI](https://arduino.cc/en/software) installed.

```bash
# Install board core (once)
arduino-cli core install arduino:renesas_uno

# Install libraries (once)
arduino-cli lib install "ArduinoMqttClient" "ArduinoJson" "Modulino"

# Detect board port
arduino-cli board list

# Compile (folder name must match .ino filename)
arduino-cli compile --fqbn arduino:renesas_uno:unor4wifi <sketch-folder>

# Upload (replace COM3 with your port)
arduino-cli upload -p COM3 --fqbn arduino:renesas_uno:unor4wifi <sketch-folder>
```

---

## Serial Monitor Output (115200 baud)

```
=== HiveMQ MQTT Sensor Station ===

Connecting to WiFi: YOUR_WIFI_SSID
WiFi: OK
IP address: 192.168.x.x
MAC address: AA:BB:CC:DD:EE:FF
Connecting to MQTT broker: YOUR_HIVEMQ_BROKER.s2.eu.hivemq.cloud
MQTT Broker: OK
Subscribed to: /AABBCCDDEEFF/alarm/status
Subscribed to: /alarmTest
Publishing to: /AABBCCDDEEFF/sensorData
Modulino I2C: OK
Modulino Thermo: OK
Modulino Buzzer: OK
XY-MD02 RS-485: OK

All services started successfully.
Publishing every 60 seconds...

Published to /AABBCCDDEEFF/sensorData: {"sensorData":{"timestamp":1741046400,"alarmStatus":"alarmOff","ModulinoThermo":[{"temperature":"25.0"},{"humidity":"60.0"}],"XYMD02":[{"temperature":"24.8"},{"humidity":"58.3"}]}}
```
