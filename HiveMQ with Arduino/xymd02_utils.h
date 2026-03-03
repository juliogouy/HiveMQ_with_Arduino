#ifndef XYMD02_UTILS_H
#define XYMD02_UTILS_H

#include <Arduino.h>
#include "config.h"

// ── CRC16 Modbus (poly 0xA001, init 0xFFFF) ─────────────────────────
static uint16_t crc16Modbus(uint8_t *buf, int len) {
  uint16_t crc = 0xFFFF;
  for (int i = 0; i < len; i++) {
    crc ^= buf[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

// ── Initialize RS-485 interface ──────────────────────────────────────
// Wiring:
//   Arduino D1 (TX1) → MAX485 DI
//   Arduino D0 (RX1) → MAX485 RO
//   Arduino D2       → MAX485 DE + RE (tied together)
//   MAX485 A / B     → XY-MD02 A+ / B-
bool initXYMD02() {
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, LOW);  // Start in receive mode
  Serial1.begin(RS485_BAUD);           // 8N1 by default
  delay(100);
  Serial.println("XY-MD02 RS-485: OK");
  return true;
}

// ── Read temperature and humidity from XY-MD02 via Modbus RTU ───────
// Sends function-code 0x04 continuous read (2 input registers from 0x0001).
// Returns true on success; fills temp (°C x 0.1 resolution) and hum (%RH x 0.1).
bool readXYMD02(float &temp, float &hum) {
  // Build 8-byte Modbus RTU request
  // Continuous read: start addr 0x0001, quantity 0x0002 (temp + humidity)
  uint8_t req[8];
  req[0] = (uint8_t)XYMD02_ADDRESS;
  req[1] = 0x04;  // Function code: read input registers
  req[2] = 0x00;  // Start address Hi
  req[3] = 0x01;  // Start address Lo  (register 0x0001 = Temperature)
  req[4] = 0x00;  // Quantity Hi
  req[5] = 0x02;  // Quantity Lo (2 registers: temp then humidity)
  uint16_t crc = crc16Modbus(req, 6);
  req[6] = crc & 0xFF;        // CRC Lo (transmitted first per Modbus spec)
  req[7] = (crc >> 8) & 0xFF; // CRC Hi

  // Flush any stale bytes in the receive buffer
  while (Serial1.available()) Serial1.read();

  // Transmit
  digitalWrite(RS485_DE_RE_PIN, HIGH);  // Enable driver (transmit)
  Serial1.write(req, 8);
  Serial1.flush();                      // Wait until all bytes are sent
  digitalWrite(RS485_DE_RE_PIN, LOW);   // Enable receiver

  // Wait for 9-byte response with 500 ms timeout
  // Response: addr(1) funcCode(1) byteCount(1) TH(1) TL(1) HH(1) HL(1) CRC_Lo(1) CRC_Hi(1)
  unsigned long start = millis();
  while (Serial1.available() < 9 && millis() - start < 500) {}

  if (Serial1.available() < 9) {
    Serial.println("XY-MD02: timeout waiting for response");
    return false;
  }

  uint8_t resp[9];
  for (int i = 0; i < 9; i++) resp[i] = Serial1.read();

  // Validate CRC over first 7 bytes; CRC in response is [7]=Lo, [8]=Hi
  uint16_t rxCrc = ((uint16_t)resp[8] << 8) | resp[7];
  if (crc16Modbus(resp, 7) != rxCrc) {
    Serial.println("XY-MD02: CRC error");
    return false;
  }

  // Validate frame header (addr, function code, byte count)
  if (resp[0] != (uint8_t)XYMD02_ADDRESS || resp[1] != 0x04 || resp[2] != 0x04) {
    Serial.println("XY-MD02: unexpected response header");
    return false;
  }

  // Parse temperature: signed int16, divide by 10 to get °C
  int16_t rawTemp = (int16_t)((resp[3] << 8) | resp[4]);
  temp = rawTemp / 10.0f;

  // Parse humidity: unsigned int16, divide by 10 to get %RH
  uint16_t rawHum = (uint16_t)((resp[5] << 8) | resp[6]);
  hum = rawHum / 10.0f;

  return true;
}

#endif
