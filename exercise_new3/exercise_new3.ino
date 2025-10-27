// Exercise 3: Read raw 8-bit bearing value from CMPS14 compass
// Print value to Serial monitor

#include <Arduino.h>
#include <Wire.h>

// CMPS14 I2C address
const uint8_t CMPS14_ADDRESS = 0x60;

// CMPS14 Register addresses
const uint8_t CMPS14_BEARING_8BIT = 0x01;  // 8-bit bearing (0-255)

// Update interval
const unsigned long UPDATE_INTERVAL_MS = 500;  // Update every 500ms

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);

  // Wait for Serial to be ready
  while (!Serial) {
    delay(10);
  }

  Serial.println("=== CMPS14 Compass - 8-bit Bearing Test ===");
  Serial.println("Initializing I2C...");

  // Initialize I2C communication
  Wire.begin();

  // Small delay to let I2C stabilize
  delay(100);

  Serial.println("I2C initialized. Reading compass data...");
  Serial.println("8-bit bearing range: 0-255 (0 = North, 64 = East, 128 = South, 192 = West)");
  Serial.println();
}

void loop() {
  // Read 8-bit bearing from CMPS14
  uint8_t bearing8bit = readCompass8Bit();

  // Print to Serial monitor
  Serial.print("8-bit Bearing: ");
  Serial.print(bearing8bit);
  Serial.print(" (0-255)");

  // Convert to approximate degrees for reference
  float degrees = (bearing8bit * 360.0f) / 255.0f;
  Serial.print(" | Approx degrees: ");
  Serial.println(degrees, 1);

  // Wait before next reading
  delay(UPDATE_INTERVAL_MS);
}

uint8_t readCompass8Bit() {
  // Request data from CMPS14 starting at register 0x01
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(CMPS14_BEARING_8BIT);  // Set register pointer to 8-bit bearing
  uint8_t error = Wire.endTransmission();

  if (error != 0) {
    Serial.print("I2C Error: ");
    Serial.println(error);
    return 0;
  }

  // Request 1 byte from CMPS14
  Wire.requestFrom(CMPS14_ADDRESS, (uint8_t)1);

  // Wait for data to be available
  unsigned long startTime = millis();
  while (Wire.available() < 1) {
    if (millis() - startTime > 100) {  // 100ms timeout
      Serial.println("Timeout waiting for compass data");
      return 0;
    }
  }

  // Read the 8-bit bearing value
  uint8_t bearing = Wire.read();

  return bearing;
}
