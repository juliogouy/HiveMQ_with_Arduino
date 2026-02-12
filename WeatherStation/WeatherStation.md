# WeatherStation

Arduino UNO R4 WiFi project that uses the Arduino Modulino Thermo and print the temperature, in celsius and the humidity to the Arduino IDE Serial Monitor running at 115200 bps speed

## Features

- **Display the Temperature in Celsius to the Serial Monitor every 15 seconds** 
- **Display the Humidity to the Serial Monitor every 15 seconds**

## Hardware

- Arduino UNO R4 WiFi
- Modulino THERMO I2C module

## Configuration

| Parameter               | Default  | Description              |
| ----------------------- | -------- | ------------------------ |
| `Serial Print Interval` | 15       | seconds                  |
| `Temperature`           | Celsius  |                          |
| `Humidity`              | Percent  |                          |
| `Serial baud`           | `115200` | Serial monitor baud rate |

## Serial Output

Once uploaded, open the Serial Monitor at 115200 baud to see:

```
Welcome to the Weather Station, once at the beggining
Temperature: xx C == Humidity yy %, wait 15 seconds, read the sensor again, display the information

```

## Upload

1. Open `WeatherStation.ino` in the Arduino IDE.
2. Select **Arduino UNO R4 WiFi** as the board.
3. Modulino THERMO I2C module.
4. Click **Upload**.
