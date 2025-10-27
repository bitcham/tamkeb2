// Exercise 4: Display compass bearing (0-360°) and directional names on LCD
// Test compass readings and display direction names

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal.h>

// CMPS14 I2C address
const uint8_t CMPS14_ADDRESS = 0x60;

// CMPS14 Register addresses
const uint8_t CMPS14_BEARING_16BIT_HIGH = 0x02;  // High byte of 16-bit bearing
const uint8_t CMPS14_BEARING_16BIT_LOW = 0x03;   // Low byte of 16-bit bearing

// LCD pins
const int LCD_RS = 53;
const int LCD_E = 51;
const int LCD_D4 = 35;
const int LCD_D5 = 34;
const int LCD_D6 = 33;
const int LCD_D7 = 32;

// Update interval
const unsigned long UPDATE_INTERVAL_MS = 300;  // Update every 300ms

// LCD object
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Compass setup
float calibrationoffset = 0.0;

void setup() {
  // Initialize Serial communication
  Serial.begin(9600);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("CMPS14 Compass");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  // Initialize I2C communication
  Wire.begin();

  // Small delay to let I2C stabilize
  delay(1000);

  lcd.clear();
  Serial.println("=== CMPS14 Compass with Direction ===");
  Serial.println("Reading compass data...");

  // Read 16-bit bearing from CMPS14 (0-3599, representing 0-359.9°)
  uint16_t bearing16bit = readCompass16Bit();

  // Calibrate
  calibrationoffset = -(bearing16bit / 10.0f);
}

void loop() {
  // Read 16-bit bearing from CMPS14 (0-3599, representing 0-359.9°)
  uint16_t bearing16bit = readCompass16Bit();

  // Convert to degrees (0-359.9)
  float beforecalibrationdegrees = bearing16bit / 10.0f;

  // Calibrate
  float degrees = beforecalibrationdegrees + calibrationoffset;
  if (degrees < 0) {degrees += 360;}
  if (degrees >= 360) {degrees -= 360;}
  
  // Get direction name
  String direction = getDirectionName(degrees);

  // Display on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Bearing: ");
  lcd.print(degrees, 1);
  lcd.print((char)223);  // Degree symbol

  lcd.setCursor(0, 1);
  lcd.print("Direction: ");
  lcd.print(direction);

  // Print to Serial
  Serial.print("Bearing: ");
  Serial.print(degrees, 1);
  Serial.print("° | Direction: ");
  Serial.println(direction);

  // Wait before next reading
  delay(UPDATE_INTERVAL_MS);
}

uint16_t readCompass16Bit() {
  // Request data from CMPS14 starting at register 0x02 (16-bit bearing)
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(CMPS14_BEARING_16BIT_HIGH);  // Set register pointer
  uint8_t error = Wire.endTransmission();

  if (error != 0) {
    Serial.print("I2C Error: ");
    Serial.println(error);
    return 0;
  }

  // Request 2 bytes from CMPS14
  Wire.requestFrom(CMPS14_ADDRESS, (uint8_t)2);

  // Wait for data to be available
  unsigned long startTime = millis();
  while (Wire.available() < 2) {
    if (millis() - startTime > 100) {  // 100ms timeout
      Serial.println("Timeout waiting for compass data");
      return 0;
    }
  }

  // Read the 16-bit bearing value (high byte first, then low byte)
  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();

  uint16_t bearing = (highByte << 8) | lowByte;

  return bearing;
}

String getDirectionName(float degrees) {
  // Direction ranges according to the exercise specification:
  // N:  338 - <23
  // NE: 23  - <68
  // E:  68  - <113
  // SE: 113 - <158
  // S:  158 - <203
  // SW: 203 - <248
  // W:  248 - <293
  // NW: 293 - <338

  if (degrees >= 338.0f || degrees < 23.0f) {
    return "N";
  } else if (degrees >= 23.0f && degrees < 68.0f) {
    return "NE";
  } else if (degrees >= 68.0f && degrees < 113.0f) {
    return "E";
  } else if (degrees >= 113.0f && degrees < 158.0f) {
    return "SE";
  } else if (degrees >= 158.0f && degrees < 203.0f) {
    return "S";
  } else if (degrees >= 203.0f && degrees < 248.0f) {
    return "SW";
  } else if (degrees >= 248.0f && degrees < 293.0f) {
    return "W";
  } else if (degrees >= 293.0f && degrees < 338.0f) {
    return "NW";
  }

  return "??";  // Should never reach here
}