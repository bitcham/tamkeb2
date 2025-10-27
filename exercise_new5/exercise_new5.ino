// Exercise 5: Turn car to face any specified heading (0-360°)
// Auto-correct if manually turned after reaching target
// Use find_heading() function with offset tolerance

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal.h>

// CMPS14 I2C address
const uint8_t CMPS14_ADDRESS = 0x60;

// CMPS14 Register addresses
const uint8_t CMPS14_BEARING_16BIT_HIGH = 0x02;
const uint8_t CMPS14_BEARING_16BIT_LOW = 0x03;

// Motor control pins
const uint8_t MOTOR_FORWARD = 1;
const uint8_t MOTOR_BACKWARD = 0;
const uint8_t MOTOR_L_DIR_PIN = 7;
const uint8_t MOTOR_R_DIR_PIN = 8;
const uint8_t MOTOR_L_PWM_PIN = 9;
const uint8_t MOTOR_R_PWM_PIN = 10;

// LCD pins
const int LCD_RS = 53;
const int LCD_E = 51;
const int LCD_D4 = 35;
const int LCD_D5 = 34;
const int LCD_D6 = 33;
const int LCD_D7 = 32;

// Target heading (hardcoded - change this to test different directions)
const float TARGET_HEADING = 90.0f;  // Example: 90° = East

// Heading tolerance (offset)
const float HEADING_OFFSET = 8.0f;  // ±8 degrees tolerance (increased to prevent oscillation)

// Turn speed
const uint8_t TURN_SPEED_PERCENT = 25;  // Lower speed for smoother turning

// Update interval
const unsigned long UPDATE_INTERVAL_MS = 200;  // Increased interval to reduce oscillation

// LCD object
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Function declarations
float readCompassDegrees();
uint16_t readCompass16Bit();
void find_heading(float given_degree);
void turnLeft(uint8_t speed);
void turnRight(uint8_t speed);
void stopMotors();
uint8_t percentToPwm(uint8_t percent);
void updateLCD(float target, float current);
float normalizeAngle(float angle);
float calculateTurnAngle(float current, float target);

void setup() {
  // Initialize Serial
  Serial.begin(9600);

  // Initialize motor pins
  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_R_PWM_PIN, OUTPUT);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Compass Turning");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  // Initialize I2C
  Wire.begin();
  delay(1000);

  lcd.clear();
  Serial.println("=== Exercise 5: Compass Heading ===");
  Serial.print("Target heading: ");
  Serial.print(TARGET_HEADING, 1);
  Serial.println("°");

  // Stop motors initially
  stopMotors();
}

void loop() {
  // Continuously try to maintain the target heading
  find_heading(TARGET_HEADING);

  delay(UPDATE_INTERVAL_MS);
}

void find_heading(float given_degree) {
  // Read current heading
  float currentHeading = readCompassDegrees();

  // Calculate the shortest turn angle
  float turnAngle = calculateTurnAngle(currentHeading, given_degree);

  // Update LCD
  updateLCD(given_degree, currentHeading);

  // Check if within tolerance
  if (abs(turnAngle) <= HEADING_OFFSET) {
    // We're at the target heading
    stopMotors();

    // Serial output only when first reaching target (not continuously)
    static bool targetReached = false;
    if (!targetReached) {
      Serial.println("Target heading reached!");
      targetReached = true;
    }
  } else {
    // Need to turn
    static bool targetReached = true;  // Reset flag
    targetReached = false;

    if (turnAngle > 0) {
      // Turn right (clockwise)
      turnRight(TURN_SPEED_PERCENT);
      Serial.print("Turning right... Current: ");
    } else {
      // Turn left (counter-clockwise)
      turnLeft(TURN_SPEED_PERCENT);
      Serial.print("Turning left... Current: ");
    }

    Serial.print(currentHeading, 1);
    Serial.print("° Target: ");
    Serial.print(given_degree, 1);
    Serial.print("° Error: ");
    Serial.print(turnAngle, 1);
    Serial.println("°");
  }
}

float readCompassDegrees() {
  uint16_t bearing16bit = readCompass16Bit();
  // Convert from 0-3599 to 0-359.9 degrees
  float degrees = bearing16bit / 10.0f;
  return degrees;
}

uint16_t readCompass16Bit() {
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(CMPS14_BEARING_16BIT_HIGH);
  uint8_t error = Wire.endTransmission();

  if (error != 0) {
    return 0;
  }

  Wire.requestFrom(CMPS14_ADDRESS, (uint8_t)2);

  unsigned long startTime = millis();
  while (Wire.available() < 2) {
    if (millis() - startTime > 100) {
      return 0;
    }
  }

  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();

  return (highByte << 8) | lowByte;
}

float calculateTurnAngle(float current, float target) {
  // Calculate the shortest turn direction and angle
  // Positive = turn right, Negative = turn left

  float diff = target - current;

  // Normalize to -180 to +180 range
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;

  return diff;
}

void turnLeft(uint8_t speed) {
  // Left wheel backward, right wheel forward (turn left in place)
  digitalWrite(MOTOR_L_DIR_PIN, MOTOR_BACKWARD);
  digitalWrite(MOTOR_R_DIR_PIN, MOTOR_FORWARD);
  uint8_t pwmValue = percentToPwm(speed);
  analogWrite(MOTOR_L_PWM_PIN, pwmValue);
  analogWrite(MOTOR_R_PWM_PIN, pwmValue);
}

void turnRight(uint8_t speed) {
  // Left wheel forward, right wheel backward (turn right in place)
  digitalWrite(MOTOR_L_DIR_PIN, MOTOR_FORWARD);
  digitalWrite(MOTOR_R_DIR_PIN, MOTOR_BACKWARD);
  uint8_t pwmValue = percentToPwm(speed);
  analogWrite(MOTOR_L_PWM_PIN, pwmValue);
  analogWrite(MOTOR_R_PWM_PIN, pwmValue);
}

void stopMotors() {
  analogWrite(MOTOR_L_PWM_PIN, 0);
  analogWrite(MOTOR_R_PWM_PIN, 0);
}

uint8_t percentToPwm(uint8_t percent) {
  percent = constrain(percent, 0, 100);
  uint8_t pwm = (uint8_t)((percent * 255UL) / 100UL);

  // Dead zone: 84-94 PWM = stop motors (prevents micro-movements)
  if (pwm >= 88 && pwm <= 92) {
    return 0;
  }

  return pwm;
}

void updateLCD(float target, float current) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Tgt:");
  lcd.print(target, 0);
  lcd.print((char)223);  // Degree symbol
  lcd.print(" Cur:");
  lcd.print(current, 0);
  lcd.print((char)223);

  lcd.setCursor(0, 1);
  float error = calculateTurnAngle(current, target);
  if (abs(error) <= HEADING_OFFSET) {
    lcd.print("ON TARGET!");
  } else {
    lcd.print("Err:");
    lcd.print(error, 1);
    lcd.print((char)223);
    lcd.print(" ");
    lcd.print(error > 0 ? "R" : "L");
  }
}

float normalizeAngle(float angle) {
  while (angle >= 360.0f) angle -= 360.0f;
  while (angle < 0.0f) angle += 360.0f;
  return angle;
}
