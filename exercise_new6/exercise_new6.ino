// Exercise 6: Triangular route with compass and encoder
// Based on working exercise_new4 code

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal.h>

// CMPS14 I2C address
const uint8_t CMPS14_ADDRESS = 0x60;
const uint8_t CMPS14_BEARING_16BIT_HIGH = 0x02;

// Motor pins
const uint8_t MOTOR_FORWARD = 1;
const uint8_t MOTOR_BACKWARD = 0;
const uint8_t MOTOR_L_DIR_PIN = 7;
const uint8_t MOTOR_R_DIR_PIN = 8;
const uint8_t MOTOR_L_PWM_PIN = 9;
const uint8_t MOTOR_R_PWM_PIN = 10;

// Encoder pins
const uint8_t ENCODER_R_PIN = 2;
const uint8_t ENCODER_L_PIN = 3;

// Joystick button
const uint8_t JOYSTICK_BUTTON_PIN = 18;

// LCD pins
const int LCD_RS = 53;
const int LCD_E = 51;
const int LCD_D4 = 35;
const int LCD_D5 = 34;
const int LCD_D6 = 33;
const int LCD_D7 = 32;

// Route parameters
const float SEGMENT1_DIST = 20.0f;
const uint8_t SEGMENT1_SPEED = 75;
const float TURN1_ANGLE = 110.0f;
const float SEGMENT2_DIST = 13.0f;
const uint8_t SEGMENT2_SPEED = 25;
const float TURN2_ANGLE = 110.0f;
const float SEGMENT3_DIST = 20.0f;
const uint8_t SEGMENT3_SPEED = 50;

// Calibration
const float PULSES_PER_CM = 84.0755f;
const float HEADING_TOLERANCE = 4.0f;
const uint8_t TURN_SPEED = 30;

// Variables
volatile unsigned long encoderLeft = 0;
volatile unsigned long encoderRight = 0;
volatile bool buttonPressed = false;
volatile unsigned long lastButtonTime = 0;

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Function declarations
uint16_t readCompass16Bit();
float readCompassDegrees();
void driveDistance(float cm, uint8_t speed);
void turnLeft(float degrees);
void motorForward(uint8_t speed);
void motorTurnLeft(uint8_t speed);
void motorTurnRight(uint8_t speed);
void motorStop();
uint8_t percentToPwm(uint8_t pct);
void encoderLeftISR();
void encoderRightISR();
void buttonISR();

void setup() {
  Serial.begin(9600);

  // Initialize LCD (same as exercise_new4)
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Exercise 6");
  lcd.setCursor(0, 1);
  lcd.print("Init...");

  // Initialize I2C (same as exercise_new4)
  Wire.begin();
  delay(1000);

  lcd.clear();
  Serial.println("=== Exercise 6 ===");

  // Setup motor pins
  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_R_PWM_PIN, OUTPUT);
  motorStop();

  // Setup encoder pins
  pinMode(ENCODER_R_PIN, INPUT);
  pinMode(ENCODER_L_PIN, INPUT);

  // Setup button
  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), encoderRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), encoderLeftISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(JOYSTICK_BUTTON_PIN), buttonISR, FALLING);

  lcd.setCursor(0, 0);
  lcd.print("Ready!");
  lcd.setCursor(0, 1);
  lcd.print("Press button");
  Serial.println("Press button");
}

void loop() {
  if (buttonPressed) {
    buttonPressed = false;

    Serial.println("\n=== Starting Route ===");
    lcd.clear();
    lcd.print("Starting...");
    delay(1000);

    Serial.println("[DEBUG] Reading compass...");
    float startHeading = readCompassDegrees();
    Serial.println("[DEBUG] Compass read OK!");
    Serial.print("Start heading: ");
    Serial.println(startHeading);

    // Segment 1
    Serial.println("[1] 20cm @ 75%");
    lcd.clear();
    lcd.print("Seg 1: 20cm");
    driveDistance(SEGMENT1_DIST, SEGMENT1_SPEED);

    // Turn 1
    Serial.println("[2] Turn left 110");
    lcd.clear();
    lcd.print("Turn left 110");
    turnLeft(TURN1_ANGLE);

    // Segment 2
    Serial.println("[3] 13cm @ 25%");
    lcd.clear();
    lcd.print("Seg 2: 13cm");
    driveDistance(SEGMENT2_DIST, SEGMENT2_SPEED);

    // Turn 2
    Serial.println("[4] Turn left 110");
    lcd.clear();
    lcd.print("Turn left 110");
    turnLeft(TURN2_ANGLE);

    // Segment 3
    Serial.println("[5] 20cm @ 50%");
    lcd.clear();
    lcd.print("Seg 3: 20cm");
    driveDistance(SEGMENT3_DIST, SEGMENT3_SPEED);

    motorStop();

    float endHeading = readCompassDegrees();
    Serial.print("End heading: ");
    Serial.println(endHeading);

    lcd.clear();
    lcd.print("DONE!");
    Serial.println("=== Route Complete ===");
  }

  delay(50);
}

void driveDistance(float cm, uint8_t speed) {
  encoderLeft = 0;
  encoderRight = 0;
  unsigned long targetPulses = (unsigned long)(cm * PULSES_PER_CM);

  motorForward(speed);

  while (encoderLeft < targetPulses) {
    delay(10);
  }

  motorStop();
  delay(200);

  Serial.print("  Drove: ");
  Serial.print(encoderLeft / PULSES_PER_CM);
  Serial.println("cm");
}

void turnLeft(float degrees) {
  float current = readCompassDegrees();
  float target = current - degrees;
  if (target < 0) target += 360.0f;

  Serial.print("  Current: ");
  Serial.print(current);
  Serial.print(" -> Target: ");
  Serial.println(target);

  unsigned long startTime = millis();

  while (millis() - startTime < 10000) {  // 10s timeout
    current = readCompassDegrees();

    float diff = target - current;
    if (diff > 180.0f) diff -= 360.0f;
    if (diff < -180.0f) diff += 360.0f;

    if (abs(diff) <= HEADING_TOLERANCE) {
      motorStop();
      delay(200);
      Serial.print("  Reached: ");
      Serial.println(current);
      return;
    }

    if (diff > 0) {
      motorTurnRight(TURN_SPEED);
    } else {
      motorTurnLeft(TURN_SPEED);
    }

    delay(50);
  }

  motorStop();
  Serial.println("  Turn timeout!");
}

void motorForward(uint8_t speed) {
  digitalWrite(MOTOR_L_DIR_PIN, MOTOR_FORWARD);
  digitalWrite(MOTOR_R_DIR_PIN, MOTOR_FORWARD);
  uint8_t pwm = percentToPwm(speed);
  analogWrite(MOTOR_L_PWM_PIN, pwm);
  analogWrite(MOTOR_R_PWM_PIN, pwm);
}

void motorTurnLeft(uint8_t speed) {
  digitalWrite(MOTOR_L_DIR_PIN, MOTOR_BACKWARD);
  digitalWrite(MOTOR_R_DIR_PIN, MOTOR_FORWARD);
  uint8_t pwm = percentToPwm(speed);
  analogWrite(MOTOR_L_PWM_PIN, pwm);
  analogWrite(MOTOR_R_PWM_PIN, pwm);
}

void motorTurnRight(uint8_t speed) {
  digitalWrite(MOTOR_L_DIR_PIN, MOTOR_FORWARD);
  digitalWrite(MOTOR_R_DIR_PIN, MOTOR_BACKWARD);
  uint8_t pwm = percentToPwm(speed);
  analogWrite(MOTOR_L_PWM_PIN, pwm);
  analogWrite(MOTOR_R_PWM_PIN, pwm);
}

void motorStop() {
  analogWrite(MOTOR_L_PWM_PIN, 0);
  analogWrite(MOTOR_R_PWM_PIN, 0);
}

uint8_t percentToPwm(uint8_t pct) {
  pct = constrain(pct, 0, 100);
  return (uint8_t)((pct * 255UL) / 100UL);
}

float readCompassDegrees() {
  uint16_t bearing = readCompass16Bit();
  return bearing / 10.0f;
}

uint16_t readCompass16Bit() {
  // Detach encoder interrupts before I2C communication
  detachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN));
  detachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN));

  // Same as exercise_new4
  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(CMPS14_BEARING_16BIT_HIGH);
  uint8_t error = Wire.endTransmission();

  if (error != 0) {
    // Re-attach interrupts before returning
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), encoderRightISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), encoderLeftISR, CHANGE);
    Serial.print("I2C Error: ");
    Serial.println(error);
    return 0;
  }

  Wire.requestFrom(CMPS14_ADDRESS, (uint8_t)2);

  unsigned long startTime = millis();
  while (Wire.available() < 2) {
    if (millis() - startTime > 100) {
      // Re-attach interrupts before returning
      attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), encoderRightISR, CHANGE);
      attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), encoderLeftISR, CHANGE);
      Serial.println("Timeout");
      return 0;
    }
  }

  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();

  // Re-attach encoder interrupts after I2C communication
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), encoderRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), encoderLeftISR, CHANGE);

  return (highByte << 8) | lowByte;
}

void encoderLeftISR() {
  encoderLeft++;
}

void encoderRightISR() {
  encoderRight++;
}

void buttonISR() {
  unsigned long now = millis();
  if (now - lastButtonTime > 300) {
    buttonPressed = true;
    lastButtonTime = now;
  }
}
