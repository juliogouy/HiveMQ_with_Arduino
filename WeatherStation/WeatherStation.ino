/*
  WeatherStation - Arduino UNO R4 WiFi + Modulino Thermo
  Hardware test: verifies Serial, Modulino, and Thermo initialization
  with a 5-second timeout for each service.
*/

#include <Modulino.h>  // Arduino_Modulino library

ModulinoThermo thermo;

const unsigned long TIMEOUT = 5000;
const unsigned long PRINT_INTERVAL = 15000;
unsigned long previousMillis = 0;
bool allOk = false;

void setup() {
  // Test Serial
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < TIMEOUT) {
    // wait for Serial or timeout
  }
  if (Serial) {
    Serial.println("Serial: OK");
  }
  // If Serial failed we can't print, but continue anyway

  // Test Modulino I2C bus
  start = millis();
  bool modulinoOk = false;
  while (millis() - start < TIMEOUT) {
    Modulino.begin();
    modulinoOk = true;
    break;
  }
  if (modulinoOk) {
    Serial.println("Modulino: OK");
  } else {
    Serial.println("Modulino: FAILED (timeout)");
  }

  // Test Thermo sensor
  start = millis();
  bool thermoOk = false;
  while (millis() - start < TIMEOUT) {
    if (thermo.begin()) {
      thermoOk = true;
      break;
    }
  }
  if (thermoOk) {
    Serial.println("Thermo: OK");
  } else {
    Serial.println("Thermo: FAILED (timeout)");
  }

  Serial.println();
  if (modulinoOk && thermoOk) {
    Serial.println("All services started successfully.");
    allOk = true;

    Serial.println("Welcome to the Weather Station");
    float temperature = thermo.getTemperature();
    float humidity = thermo.getHumidity();
    Serial.print("Temperature: ");
    Serial.print(temperature, 1);
    Serial.print(" C == Humidity ");
    Serial.print(humidity, 1);
    Serial.println(" %");
  } else {
    Serial.println("One or more services failed to start.");
  }
}

void loop() {
  if (!allOk) return;

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= PRINT_INTERVAL) {
    previousMillis = currentMillis;

    float temperature = thermo.getTemperature();
    float humidity = thermo.getHumidity();
    Serial.print("Temperature: ");
    Serial.print(temperature, 1);
    Serial.print(" C == Humidity ");
    Serial.print(humidity, 1);
    Serial.println(" %");
  }
}
