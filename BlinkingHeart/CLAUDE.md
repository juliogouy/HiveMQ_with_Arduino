# CLAUDE.md

## Project Overview

Arduino UNO R4 WiFi project that blinks a filled heart on the built-in 12x8 LED matrix (red only, single color) and the built-in LED, while connecting to WiFi.

## Tech Stack

- **Board**: Arduino UNO R4 WiFi
- **Library**: WiFiS3 (built-in for UNO R4 WiFi)
- **Language**: C++ (Arduino)

## Project Structure

- `BlinkingHeart.ino` — Main sketch (WiFi connection + non-blocking LED blink + Blinking Heart on the LED Matrix)
- `BlinkHeart.md` — Project documentation (Typora/Markdown)

## Conventions

- Use `millis()` for timing instead of `delay()` to keep the loop non-blocking
- WiFi credentials are defined as constants at the top of the sketch
- Serial output at 115200 baud for debugging
- Wait for valid DHCP IP before proceeding after WiFi connection
- The built-in LED matrix is red-only (single color); the heart blinks on/off
- Use `Arduino_LED_Matrix.h` with `loadFrame()` for matrix rendering

## Build & Upload

Open `BlinkingHeart.ino` in Arduino IDE, select **Arduino UNO R4 WiFi** as the board, and upload.
