#ifndef WIFI_UTILS_H
#define WIFI_UTILS_H

#include <WiFiS3.h>
#include "config.h"

// ── Build a short human-readable device ID from the MAC address ─────
// Uses FNV-1a 32-bit hash of the 6 MAC bytes, truncated to 24 bits
// (6 lowercase hex chars). Result: "station/<xxxxxx>"
// Same board always produces the same ID; ~16M possible values.
String buildDeviceId() {
  byte mac[6];
  WiFi.macAddress(mac);
  uint32_t hash = 2166136261UL;      // FNV-1a offset basis
  for (int i = 0; i < 6; i++) {
    hash ^= (uint32_t)mac[i];
    hash *= 16777619UL;              // FNV-1a prime
  }
  char suffix[7];
  snprintf(suffix, sizeof(suffix), "%06x", hash & 0x00FFFFFFU);
  return String("station/") + suffix;
}

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

#endif
