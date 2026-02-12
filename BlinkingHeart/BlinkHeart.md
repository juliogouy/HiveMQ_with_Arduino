# BlinkHeart

Arduino UNO R4 WiFi project that displays a blinking heart on the built-in 12x8 LED matrix while connecting to WiFi.

## Features

- Filled heart shape blinks on/off on the red LED matrix at 500ms intervals
- Built-in LED blinks in sync with the heart
- Connects to WiFi and prints the assigned IP address over Serial
- Non-blocking timing using `millis()`

## Configuration

Edit the constants at the top of `BlinkingHeart.ino`:

- `WIFI_SSID` / `WIFI_PASS` — WiFi credentials
- `BLINK_INTERVAL` — blink speed in milliseconds (default: 500)

## Upload

1. Open `BlinkingHeart.ino` in Arduino IDE
2. Select **Arduino UNO R4 WiFi** as the board
3. Select the correct serial port
4. Upload

## Serial Output

Open Serial Monitor at **115200 baud** to see WiFi connection status and IP address.
