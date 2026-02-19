# HiveMQ_Project with C++

Arduino Uno R4 WiFi project that will connect to the MQTT Broker HiveMQ to publish a test topic

## Features

- **Publish an MQTT topic "/sensorData" every 60 seconds** 

## Hardware

- Arduino UNO R4 WiFi on COM4

## Configuration

| Parameter                  | Default topic to publish creation                            | Description                                                  |
| -------------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| `topic`                    | /"wifi board mac address"/sensorData/"json object"           | Publish sensor data every 60 seconds. Format the topic to public this data, temperature and humidity as a json object using the WiFi mac address as part of the MQTT topic |
| MQTT Broker URL            | YOUR_HIVEMQ_BROKER.s2.eu.hivemq.cloud                       |                                                              |
| MQTT Broker Port           | 8883                                                         |                                                              |
| MQTT Broker User           | YOUR_MQTT_USERNAME                                           |                                                              |
| MQTT Broker User  Password | YOUR_MQTT_PASSWORD                                           |                                                              |
| WiFi SSID                  | YOUR_WIFI_SSID                                               |                                                              |
| WiFi Password              | YOUR_WIFI_PASSWORD                                           |                                                              |
| topic                      | /AlarmData, maybe on or off depends on the logic, if temperature is iqual or lower than 35 degress, alarm "off" if the temperature is over 35 degress, the alarm will be triggered by the /alarm/"status" topic. For test purposes, if the topic "/alarmTest" is received with "teston" content, please make the Modulino Buzzer generate an SOS signal with .250 second each short tone, .750 seconds each long tone and .5 seconds between tones. | Subscribe to the topic /alarm/"status": on, off              |

##json Data Format

Jason Object format to be published:

The timestamp should be of the **Epoch Time (Integer/Long)** format

```json
{
  "sensorData": {
    "ModulinoThermo": [
      {"timestamp": "timestamp"},
      {"temperature": "temperature"},
      {"humidity": "humidity"},
      {"alarmStatus": "alarm status"}
    ]
  }
}
```



Hardware:

1) Arduino Uno R4 WiFi
2) Modulino Termo (get temperature and humidity and fill the "temperature" and "humidity" json object respsctively)
3) Modulino Buzzer (Alarm triggered when temperature is over 35 degress and filled on the "alarmStatus" of the json obect")

Upload

1. Create an Arduino application to achieve the above task.
2. Shows the IP address received by the DHCP server and wait until it is available to be shown
3. shows the successful or not of wifi connections, MQTT Broker and modules activation
4. shows the MQTT topics that will  subscribed and published
5. if temperature is iqual or bellow 35 degress, fill jason object "alarmStatus" as of, otherwise, "alarmOn"
6. Upload to the created Arduino sketch Uni R4 WiFi
