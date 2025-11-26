/*
 * Final Assignment - Integrated Car Control System
 * Tampere University of Applied Sciences
 *
 * UPDATED:
 * - Removed joystick button
 * - Added new toggle button on pin 4 for switching ESP/Joystick modes
 */

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal.h>

// ============== PIN DEFINITIONS ==============

// Motor control pins
const uint8_t MOTOR_FORWARD = 1;
const uint8_t MOTOR_BACKWARD = 0;
const uint8_t MOTOR_L_DIR_PIN = 7;
const uint8_t MOTOR_R_DIR_PIN = 8;
const uint8_t MOTOR_L_PWM_PIN = 9;
const uint8_t MOTOR_R_PWM_PIN = 10;

// Encoder pins
const uint8_t ENCODER_R_PIN = 2;
const uint8_t ENCODER_L_PIN = 3;

// Joystick pins
const uint8_t JOYSTICK_X_PIN = A8;  
const uint8_t JOYSTICK_Y_PIN = A9;

// NEW MODE BUTTON
const uint8_t MODE_BUTTON_PIN = 4;

// LCD pins
const int LCD_RS = 53;
const int LCD_E = 51;
const int LCD_D4 = 35;
const int LCD_D5 = 34;
const int LCD_D6 = 33;
const int LCD_D7 = 32;

// ============== COMPASS SETTINGS ==============

const uint8_t CMPS14_ADDRESS = 0x60;
const uint8_t CMPS14_BEARING_16BIT_HIGH = 0x02;
const float HEADING_TOLERANCE = 5.0f;
const uint8_t TURN_SPEED = 30;

// ============== CALIBRATION VALUES ==============

const float PULSES_PER_CM = 84.0755f;
const uint8_t DRIVE_SPEED = 35;
const int JOYSTICK_CENTER = 512;
const int JOYSTICK_DEADZONE = 50;
const float PULSES_PER_CM_R_CALI = 2.16f;

// ============== GLOBAL VARIABLES ==============

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

volatile unsigned long encoderLeftPulses = 0;
volatile unsigned long encoderRightPulses = 0;

float totalDistanceLeft = 0.0f;
float totalDistanceRight = 0.0f;

bool espMode = true;

float compassOffset = 0.0f;
bool compassConnected = false;

String lastESPCommand = "None";
float targetHeading = 0.0f;
float targetDistance = 0.0f;
bool hasNewCommand = false;
String commandType = "";

unsigned long lastHeartbeatTime = 0;
const unsigned long HEARTBEAT_TIMEOUT = 5000;
bool heartbeatEnabled = false;

unsigned long lastLCDUpdate = 0;
const unsigned long LCD_UPDATE_INTERVAL = 200;

// Button debounce
bool lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
const unsigned long DEBOUNCE_DELAY = 200;

// ============== FUNCTION DECLARATIONS ==============
float readCompassRaw();
float readCompassCalibrated();
uint16_t readCompass16Bit();
String getDirectionName(float degrees);
bool initCompass();

void motorForward(uint8_t speed);
void motorBackward(uint8_t speed);
void motorTurnLeft(uint8_t speed);
void motorTurnRight(uint8_t speed);
void motorStop();
void drive(float distanceCm, bool forward, uint8_t speed);
void turn(float targetDegree, uint8_t speed);
void findNorth();
uint8_t percentToPwm(uint8_t percent);

void handleJoystickControl();
int readJoystickX();
int readJoystickY();

void readESPCommands();
void executeESPCommand();

void updateLCD();
void displayError(String errorMsg);

void encoderLeftISR();
void encoderRightISR();

// ============== SETUP ==============

void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
  Serial1.setTimeout(100);

  Wire.begin();

  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_R_PWM_PIN, OUTPUT);
  motorStop();

  pinMode(ENCODER_R_PIN, INPUT_PULLUP);
  pinMode(ENCODER_L_PIN, INPUT_PULLUP);

  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);

  // External Push Button
  pinMode(MODE_BUTTON_PIN, INPUT_PULLUP);

  lcd.begin(20, 4);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Final Assignment");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  delay(1000);

  if (initCompass()) {
    compassConnected = true;
    float initialReading = readCompassRaw();
    compassOffset = -initialReading;

    if (compassOffset < 0) compassOffset += 360.0f;
    if (compassOffset >= 360) compassOffset -= 360.0f;

    Serial1.print("Compass calibrated. Offset: ");
    Serial1.println(compassOffset);
  } else {
    compassConnected = false;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("COMPASS ERROR!");
    lcd.setCursor(0, 1);
    lcd.print("Joystick only");
    Serial1.println("ERROR: Compass not detected! Switching to Joystick mode only.");
    espMode = false;
    delay(2000);
  }

  attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), encoderRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), encoderLeftISR, CHANGE);
 

  lcd.clear();
  Serial1.println("=== Final Assignment Ready ===");
}

// ============== MAIN LOOP ==============

void loop() {

  // ========== External Push Button ==========
  bool reading = digitalRead(MODE_BUTTON_PIN);
  
  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY) {

    if (reading == LOW) {
      espMode = true;
      Serial1.print("Mode switched to: ");
      Serial1.println("ESP");
    }
    else{
      espMode = false;
      Serial1.print("Mode switched to: ");
      Serial1.println("Joystick");
    }
  }

  lastButtonState = reading;

  // ========== MODES ==========
  if (espMode) {
    readESPCommands();

    if (hasNewCommand) {
      executeESPCommand();
      hasNewCommand = false;
    }

    if (heartbeatEnabled && (millis() - lastHeartbeatTime > HEARTBEAT_TIMEOUT)) {
      motorStop();
      displayError("No heartbeat!");
      delay(500);
    }

  } else {
    handleJoystickControl();
  }

  if (millis() - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
    updateLCD();
    lastLCDUpdate = millis();
  }
}

// ============== COMPASS FUNCTIONS ==============

bool initCompass() {
  Wire.beginTransmission(CMPS14_ADDRESS);
  return (Wire.endTransmission() == 0);
}

float readCompassRaw() {
  return readCompass16Bit() / 10.0f;
}

float readCompassCalibrated() {
  float raw = readCompassRaw();
  float calibrated = raw + compassOffset;

  if (calibrated < 0) calibrated += 360.0f;
  if (calibrated >= 360) calibrated -= 360.0f;

  return calibrated;
}

uint16_t readCompass16Bit() {
  detachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN));
  detachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN));

  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(CMPS14_BEARING_16BIT_HIGH);
  Wire.endTransmission();

  Wire.requestFrom(CMPS14_ADDRESS, (uint8_t)2);
  while (Wire.available() < 2);

  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();

  attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), encoderRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), encoderLeftISR, CHANGE);

  return (highByte << 8) | lowByte;
}

String getDirectionName(float degrees) {
  if (degrees >= 338.0f || degrees < 23.0f) return "N";
  if (degrees < 68.0f) return "NE";
  if (degrees < 113.0f) return "E";
  if (degrees < 158.0f) return "SE";
  if (degrees < 203.0f) return "S";
  if (degrees < 248.0f) return "SW";
  if (degrees < 293.0f) return "W";
  return "NW";
}

// ============== MOTOR CONTROL FUNCTIONS ==============

void motorForward(uint8_t speed) {
  digitalWrite(MOTOR_L_DIR_PIN, MOTOR_FORWARD);
  digitalWrite(MOTOR_R_DIR_PIN, MOTOR_FORWARD);
  uint8_t pwm = percentToPwm(speed);
  analogWrite(MOTOR_L_PWM_PIN, pwm);
  analogWrite(MOTOR_R_PWM_PIN, pwm);
}

void motorBackward(uint8_t speed) {
  digitalWrite(MOTOR_L_DIR_PIN, MOTOR_BACKWARD);
  digitalWrite(MOTOR_R_DIR_PIN, MOTOR_BACKWARD);
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

uint8_t percentToPwm(uint8_t percent) {
  return (uint8_t)((constrain(percent, 0, 100) * 255UL) / 100UL);
}

void drive(float distanceCm, bool forward, uint8_t speed) {
  unsigned long startLeft = encoderLeftPulses;
  unsigned long targetPulses = abs(distanceCm) * PULSES_PER_CM;

  if (forward) motorForward(speed);
  else motorBackward(speed);

  while ((encoderLeftPulses - startLeft) < targetPulses) {
    if (millis() - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
      updateLCD();
      lastLCDUpdate = millis();
    }
    delay(10);
  }

  motorStop();
  totalDistanceLeft += (encoderLeftPulses - startLeft) / PULSES_PER_CM;

  delay(200);
}

void turn(float targetDegree, uint8_t speed) {
  if (!compassConnected) {
    displayError("No compass!");
    return;
  }

  while (targetDegree < 0) targetDegree += 360;
  while (targetDegree >= 360) targetDegree -= 360;

  unsigned long startTime = millis();

  while (millis() - startTime < 10000) {
    float current = readCompassCalibrated();

    float diff = targetDegree - current;
    while (diff > 180) diff -= 360;
    while (diff < -180) diff += 360;

    if (abs(diff) <= HEADING_TOLERANCE) {
      motorStop();
      return;
    }

    if (diff > 0) motorTurnRight(speed);
    else motorTurnLeft(speed);

    if (millis() - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
      updateLCD();
      lastLCDUpdate = millis();
    }

    delay(50);
  }

  motorStop();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Turn timeout");
  delay(2000);
}

void findNorth() {
  turn(0.0f, TURN_SPEED);
}

// ============== JOYSTICK CONTROL ==============

int readJoystickX() {
  int raw = analogRead(JOYSTICK_X_PIN);
  int centered = raw - JOYSTICK_CENTER;
  if (abs(centered) < JOYSTICK_DEADZONE) return 0;
  return map(centered, -512, 512, -100, 100);
}

int readJoystickY() {
  int raw = analogRead(JOYSTICK_Y_PIN);
  int centered = raw - JOYSTICK_CENTER;
  if (abs(centered) < JOYSTICK_DEADZONE) return 0;
  return map(centered, -512, 512, 100, -100);
}

void handleJoystickControl() {
  int x = readJoystickX();
  int y = readJoystickY();

  if (x == 0 && y == 0) {
    motorStop();
    return;
  }

  float magnitude = sqrt(x * x + y * y);
  uint8_t speed = constrain(magnitude, 0, 100);

  if (abs(y) > abs(x)) {
    uint8_t leftSpeed = speed;
    uint8_t rightSpeed = speed;

    if (y > 0) {
      if (x > 0) rightSpeed = speed * (100 - abs(x)) / 100;
      else if (x < 0) leftSpeed = speed * (100 - abs(x)) / 100;

      digitalWrite(MOTOR_L_DIR_PIN, MOTOR_FORWARD);
      digitalWrite(MOTOR_R_DIR_PIN, MOTOR_FORWARD);
    } else {
      if (x > 0) rightSpeed = speed * (100 - abs(x)) / 100;
      else if (x < 0) leftSpeed = speed * (100 - abs(x)) / 100;

      digitalWrite(MOTOR_L_DIR_PIN, MOTOR_BACKWARD);
      digitalWrite(MOTOR_R_DIR_PIN, MOTOR_BACKWARD);
    }

    analogWrite(MOTOR_L_PWM_PIN, percentToPwm(leftSpeed));
    analogWrite(MOTOR_R_PWM_PIN, percentToPwm(rightSpeed));

  } else {
    if (x > 0) motorTurnRight(speed);
    else motorTurnLeft(speed);
  }
}

// ============== ESP COMMAND HANDLING ==============

void readESPCommands() {
  while (Serial1.available() > 0) {
    String command = Serial1.readStringUntil('\n');
    command.trim();

    if (command == "ping") {
      lastHeartbeatTime = millis();
      heartbeatEnabled = true;
      Serial1.println("pong");
      continue;
    }

    if (command.startsWith("dist:")) {
      targetDistance = command.substring(5).toFloat();
      commandType = "dist";
      hasNewCommand = true;
      lastESPCommand = command;
    }

    else if (command.startsWith("deg:")) {
      targetHeading = command.substring(4).toFloat();
      while (targetHeading < 0) targetHeading += 360;
      while (targetHeading >= 360) targetHeading -= 360;

      commandType = "deg";
      hasNewCommand = true;
      lastESPCommand = command;
    }

    else if (command == "north") {
      commandType = "north";
      hasNewCommand = true;
      lastESPCommand = command;
    }
  }
}

void executeESPCommand() {
  if (commandType == "dist") {
    drive(abs(targetDistance), targetDistance >= 0, DRIVE_SPEED);
  }
  else if (commandType == "deg") {
    turn(targetHeading, TURN_SPEED);
  }
  else if (commandType == "north") {
    findNorth();
  }

  commandType = "";
}

// ============== LCD DISPLAY ==============

void updateLCD() {
  lcd.clear();

  lcd.setCursor(0, 0);
  lcd.print(espMode ? "ESP " : "JOY ");

  if (compassConnected) {
    float heading = readCompassCalibrated();
    lcd.print((int)heading);
    lcd.print((char)223);
    lcd.print(getDirectionName(heading));
  } else {
    lcd.print("NO CMP");
  }

  lcd.setCursor(0, 1);
  if (espMode) {
    lcd.print("C:");
    lcd.print(lastESPCommand.substring(0, 16));
  } else {
    lcd.print("X:");
    lcd.print(readJoystickX());
    lcd.print(" Y:");
    lcd.print(readJoystickY());
  }

  lcd.setCursor(0, 2);
  lcd.print("P L:");
  lcd.print(encoderLeftPulses);
  lcd.print(" R:");
  lcd.print(encoderRightPulses);

  lcd.setCursor(0, 3);
  lcd.print("D L:");
  lcd.print((int)encoderLeftPulses / PULSES_PER_CM);
  lcd.print(" R:");
  lcd.print((int)encoderRightPulses / PULSES_PER_CM * PULSES_PER_CM_R_CALI);
  lcd.print("cm");
}

void displayError(String errorMsg) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ERROR:");
  lcd.setCursor(0, 1);
  lcd.print(errorMsg);
}

// ============== INTERRUPTS ==============

void encoderLeftISR() {
  encoderLeftPulses++;
}

void encoderRightISR() {
  encoderRightPulses++;
}
