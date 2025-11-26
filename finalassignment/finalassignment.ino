/*
 * Final Assignment - Integrated Car Control System
 * Tampere University of Applied Sciences
 *
 * Features:
 * - Joystick control with variable speed and direction
 * - ESP webpage control via Serial1 (UART)
 * - Compass-based turning with calibration
 * - Encoder-based distance measurement
 * - Mode switching between Joystick and ESP mode
 * - Find North functionality
 * - LCD display showing all required information
 * - Error handling for compass connection
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

// Encoder pins (must support interrupts)
const uint8_t ENCODER_R_PIN = 5;
const uint8_t ENCODER_L_PIN = 3;

// Joystick pins
const uint8_t JOYSTICK_X_PIN = A8;  // Left-Right axis
const uint8_t JOYSTICK_Y_PIN = A9;  // Forward-Backward axis
const uint8_t JOYSTICK_BUTTON_PIN = 2;  // Mode switch button (interrupt capable)

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
const float HEADING_TOLERANCE = 5.0f;  // ±5 degree tolerance for turning
const uint8_t TURN_SPEED = 30;  // Turn speed percentage

// ============== CALIBRATION VALUES ==============

const float PULSES_PER_CM = 84.0755f;  // Encoder pulses per centimeter
const uint8_t DRIVE_SPEED = 35;  // Default drive speed percentage

// Joystick deadzone and center values
const int JOYSTICK_CENTER = 512;
const int JOYSTICK_DEADZONE = 50;

// ============== GLOBAL VARIABLES ==============

// LCD object
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Encoder pulse counters
volatile unsigned long encoderLeftPulses = 0;
volatile unsigned long encoderRightPulses = 0;

// Total distance traveled (persistent across commands)
float totalDistanceLeft = 0.0f;
float totalDistanceRight = 0.0f;

// Mode control: true = ESP mode, false = Joystick mode
volatile bool espMode = false;
volatile bool modeButtonPressed = false;
volatile unsigned long lastButtonTime = 0;
const unsigned long DEBOUNCE_DELAY = 300;

// Compass calibration offset (calculated at startup)
float compassOffset = 0.0f;
bool compassConnected = false;

// ESP command storage
String lastESPCommand = "None";
float targetHeading = 0.0f;
float targetDistance = 0.0f;
bool hasNewCommand = false;
String commandType = "";  // "dist", "deg", or "north"

// Heartbeat monitoring
unsigned long lastHeartbeatTime = 0;
const unsigned long HEARTBEAT_TIMEOUT = 5000;  // 5 seconds timeout
bool heartbeatEnabled = false;

// LCD update timing
unsigned long lastLCDUpdate = 0;
const unsigned long LCD_UPDATE_INTERVAL = 200;

// ============== FUNCTION DECLARATIONS ==============

// Compass functions
float readCompassRaw();
float readCompassCalibrated();
uint16_t readCompass16Bit();
String getDirectionName(float degrees);
bool initCompass();

// Motor control functions
void motorForward(uint8_t speed);
void motorBackward(uint8_t speed);
void motorTurnLeft(uint8_t speed);
void motorTurnRight(uint8_t speed);
void motorStop();
void drive(float distanceCm, bool forward, uint8_t speed);
void turn(float targetDegree, uint8_t speed);
void findNorth();
uint8_t percentToPwm(uint8_t percent);

// Joystick functions
void handleJoystickControl();
int readJoystickX();
int readJoystickY();

// ESP command functions
void readESPCommands();
void executeESPCommand();

// Display functions
void updateLCD();
void displayError(String errorMsg);

// Interrupt Service Routines
void encoderLeftISR();
void encoderRightISR();
void modeButtonISR();

// ============== SETUP ==============

void setup() {
  // Initialize Serial1 communication for ESP
  Serial1.begin(115200);
  Serial.begin(115200);
  Serial1.setTimeout(100);

  // Initialize I2C for compass
  Wire.begin();

  // Initialize motor pins
  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_R_PWM_PIN, OUTPUT);
  motorStop();

  // Initialize encoder pins
  pinMode(ENCODER_R_PIN, INPUT);
  pinMode(ENCODER_L_PIN, INPUT);

  // Initialize joystick pins
  pinMode(JOYSTICK_X_PIN, INPUT);
  pinMode(JOYSTICK_Y_PIN, INPUT);
  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);

  // Initialize LCD
  lcd.begin(20, 4);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Final Assignment");
  lcd.setCursor(0, 1);
  lcd.print("Initializing...");

  delay(1000);

  // Initialize and calibrate compass
  // GUIDELINE 1: Compass calibration - calculate offset so North = 0
  if (initCompass()) {
    compassConnected = true;
    float initialReading = readCompassRaw();
    compassOffset = -initialReading;  // Offset to make current direction = 0 (North)

    // Normalize offset to 0-360 range
    if (compassOffset < 0) compassOffset += 360.0f;
    if (compassOffset >= 360) compassOffset -= 360.0f;

    Serial1.print("Compass calibrated. Offset: ");
    Serial1.println(compassOffset);
  } else {
    // ERROR HANDLING: Compass connection failed
    compassConnected = false;
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("COMPASS ERROR!");
    lcd.setCursor(0, 1);
    lcd.print("Joystick only");
    Serial1.println("ERROR: Compass not detected! Switching to Joystick mode only.");
    espMode = false;  // Force joystick mode
    delay(2000);
  }

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), encoderRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), encoderLeftISR, CHANGE);

  // Attach mode button interrupt
  attachInterrupt(digitalPinToInterrupt(JOYSTICK_BUTTON_PIN), modeButtonISR, FALLING);

  // Initial display
  lcd.clear();
  Serial1.println("=== Final Assignment Ready ===");
  Serial1.println("Commands: dist:<cm>, deg:<angle>, north");
  Serial1.println("Press joystick button to switch modes");

  lastHeartbeatTime = millis();
}

// ============== MAIN LOOP ==============

void loop() {
  // Check for mode switch button press
  if (modeButtonPressed) {
    modeButtonPressed = false;

    // Only allow ESP mode if compass is connected
    if (!compassConnected && !espMode) {
      // Cannot switch to ESP mode without compass
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ESP mode needs");
      lcd.setCursor(0, 1);
      lcd.print("compass!");
      delay(1000);
    } else {
      espMode = !espMode;
      motorStop();
      Serial1.print("Mode switched to: ");
      Serial1.println(espMode ? "ESP" : "Joystick");
    }
  }

  // Handle control based on current mode
  if (espMode) {
    // ESP Mode: Read and execute commands from Serial1
    readESPCommands();

    if (hasNewCommand) {
      executeESPCommand();
      hasNewCommand = false;
    }

    // Heartbeat monitoring (ERROR HANDLING)
    if (heartbeatEnabled && (millis() - lastHeartbeatTime > HEARTBEAT_TIMEOUT)) {
      // No heartbeat received - stop motors for safety
      motorStop();
      displayError("No heartbeat!");
      delay(500);
    }
  } else {
    // Joystick Mode: Direct control
    handleJoystickControl();
  }

  // Update LCD display periodically
  if (millis() - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
    updateLCD();
    lastLCDUpdate = millis();
  }
}

// ============== COMPASS FUNCTIONS ==============

/**
 * Initialize compass and check connection
 * Returns true if compass is detected
 */
bool initCompass() {
  Wire.beginTransmission(CMPS14_ADDRESS);
  uint8_t error = Wire.endTransmission();

  if (error == 0) {
    // Try reading a value to confirm
    uint16_t testValue = readCompass16Bit();
    return true;
  }
  return false;
}

/**
 * Read raw compass value (0-359.9 degrees)
 * Without calibration offset
 */
float readCompassRaw() {
  uint16_t bearing = readCompass16Bit();
  return bearing / 10.0f;
}

/**
 * Read calibrated compass value
 * GUIDELINE 1: Applies offset so North = 0
 */
float readCompassCalibrated() {
  float raw = readCompassRaw();
  float calibrated = raw + compassOffset;

  // Normalize to 0-360 range
  if (calibrated < 0) calibrated += 360.0f;
  if (calibrated >= 360) calibrated -= 360.0f;

  return calibrated;
}

/**
 * Read 16-bit bearing value from CMPS14 compass
 * Returns value 0-3599 (representing 0-359.9 degrees)
 */
uint16_t readCompass16Bit() {
  // Temporarily disable encoder interrupts during I2C communication
  detachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN));
  detachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN));

  Wire.beginTransmission(CMPS14_ADDRESS);
  Wire.write(CMPS14_BEARING_16BIT_HIGH);
  uint8_t error = Wire.endTransmission();

  if (error != 0) {
    // Re-attach interrupts and return error
    attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), encoderRightISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), encoderLeftISR, CHANGE);
    return 0;
  }

  Wire.requestFrom(CMPS14_ADDRESS, (uint8_t)2);

  unsigned long startTime = millis();
  while (Wire.available() < 2) {
    if (millis() - startTime > 100) {
      // Timeout - re-attach interrupts and return error
      attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), encoderRightISR, CHANGE);
      attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), encoderLeftISR, CHANGE);
      return 0;
    }
  }

  uint8_t highByte = Wire.read();
  uint8_t lowByte = Wire.read();

  // Re-attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), encoderRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), encoderLeftISR, CHANGE);

  return (highByte << 8) | lowByte;
}

/**
 * Get cardinal/intercardinal direction name from degrees
 */
String getDirectionName(float degrees) {
  if (degrees >= 338.0f || degrees < 23.0f) return "N";
  else if (degrees >= 23.0f && degrees < 68.0f) return "NE";
  else if (degrees >= 68.0f && degrees < 113.0f) return "E";
  else if (degrees >= 113.0f && degrees < 158.0f) return "SE";
  else if (degrees >= 158.0f && degrees < 203.0f) return "S";
  else if (degrees >= 203.0f && degrees < 248.0f) return "SW";
  else if (degrees >= 248.0f && degrees < 293.0f) return "W";
  else if (degrees >= 293.0f && degrees < 338.0f) return "NW";
  return "?";
}

// ============== MOTOR CONTROL FUNCTIONS ==============

/**
 * Drive motors forward at specified speed
 */
void motorForward(uint8_t speed) {
  digitalWrite(MOTOR_L_DIR_PIN, MOTOR_FORWARD);
  digitalWrite(MOTOR_R_DIR_PIN, MOTOR_FORWARD);
  uint8_t pwm = percentToPwm(speed);
  analogWrite(MOTOR_L_PWM_PIN, pwm);
  analogWrite(MOTOR_R_PWM_PIN, pwm);
}

/**
 * Drive motors backward at specified speed
 */
void motorBackward(uint8_t speed) {
  digitalWrite(MOTOR_L_DIR_PIN, MOTOR_BACKWARD);
  digitalWrite(MOTOR_R_DIR_PIN, MOTOR_BACKWARD);
  uint8_t pwm = percentToPwm(speed);
  analogWrite(MOTOR_L_PWM_PIN, pwm);
  analogWrite(MOTOR_R_PWM_PIN, pwm);
}

/**
 * Turn left in place (left wheel backward, right wheel forward)
 */
void motorTurnLeft(uint8_t speed) {
  digitalWrite(MOTOR_L_DIR_PIN, MOTOR_BACKWARD);
  digitalWrite(MOTOR_R_DIR_PIN, MOTOR_FORWARD);
  uint8_t pwm = percentToPwm(speed);
  analogWrite(MOTOR_L_PWM_PIN, pwm);
  analogWrite(MOTOR_R_PWM_PIN, pwm);
}

/**
 * Turn right in place (left wheel forward, right wheel backward)
 */
void motorTurnRight(uint8_t speed) {
  digitalWrite(MOTOR_L_DIR_PIN, MOTOR_FORWARD);
  digitalWrite(MOTOR_R_DIR_PIN, MOTOR_BACKWARD);
  uint8_t pwm = percentToPwm(speed);
  analogWrite(MOTOR_L_PWM_PIN, pwm);
  analogWrite(MOTOR_R_PWM_PIN, pwm);
}

/**
 * Stop all motors
 */
void motorStop() {
  analogWrite(MOTOR_L_PWM_PIN, 0);
  analogWrite(MOTOR_R_PWM_PIN, 0);
}

/**
 * Convert percentage (0-100) to PWM value (0-255)
 */
uint8_t percentToPwm(uint8_t percent) {
  percent = constrain(percent, 0, 100);
  return (uint8_t)((percent * 255UL) / 100UL);
}

/**
 * Drive a specific distance using encoder feedback
 * GUIDELINE 2: Distance based on encoder pulses
 */
void drive(float distanceCm, bool forward, uint8_t speed) {
  // Reset pulse counters for this drive
  unsigned long startLeftPulses = encoderLeftPulses;
  unsigned long startRightPulses = encoderRightPulses;

  unsigned long targetPulses = (unsigned long)(abs(distanceCm) * PULSES_PER_CM);

  Serial1.print("Driving ");
  Serial1.print(distanceCm);
  Serial1.print("cm ");
  Serial1.println(forward ? "forward" : "backward");

  // Start driving
  if (forward) {
    motorForward(speed);
  } else {
    motorBackward(speed);
  }

  // Drive until target distance reached
  while ((encoderLeftPulses - startLeftPulses) < targetPulses) {
    // Update LCD while driving
    if (millis() - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
      updateLCD();
      lastLCDUpdate = millis();
    }
    delay(10);
  }

  motorStop();

  // Calculate actual distance traveled
  float actualDistance = (encoderLeftPulses - startLeftPulses) / PULSES_PER_CM;

  // Update total distance
  totalDistanceLeft += actualDistance;
  totalDistanceRight += (encoderRightPulses - startRightPulses) / PULSES_PER_CM;

  Serial1.print("Drive complete. Actual: ");
  Serial1.print(actualDistance);
  Serial1.println("cm");

  delay(200);
}

/**
 * Turn to face a specific heading using compass
 * GUIDELINE 2: Compass-based turning within ±5° tolerance
 */
void turn(float targetDegree, uint8_t speed) {
  if (!compassConnected) {
    displayError("No compass!");
    return;
  }

  // Normalize target to 0-360
  while (targetDegree < 0) targetDegree += 360.0f;
  while (targetDegree >= 360) targetDegree -= 360.0f;

  Serial1.print("Turning to: ");
  Serial1.print(targetDegree);
  Serial1.println(" degrees");

  unsigned long startTime = millis();
  const unsigned long TURN_TIMEOUT = 10000;  // 10 second timeout

  while (millis() - startTime < TURN_TIMEOUT) {
    float currentHeading = readCompassCalibrated();

    // Calculate shortest turn direction
    float diff = targetDegree - currentHeading;
    while (diff > 180.0f) diff -= 360.0f;
    while (diff < -180.0f) diff += 360.0f;

    // Check if within tolerance
    if (abs(diff) <= HEADING_TOLERANCE) {
      motorStop();
      Serial1.print("Turn complete. Current heading: ");
      Serial1.println(currentHeading);
      return;
    }

    // Turn in appropriate direction
    if (diff > 0) {
      motorTurnRight(speed);
    } else {
      motorTurnLeft(speed);
    }

    // Update LCD while turning
    if (millis() - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
      updateLCD();
      lastLCDUpdate = millis();
    }

    delay(50);
  }

  motorStop();
  Serial.println("Turn timeout!");
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Turn timeout");
  delay(2000);
}

/**
 * Rotate to face North (0 degrees)
 * GUIDELINE 2: Find North button functionality
 */
void findNorth() {
  Serial1.println("Finding North...");
  turn(0.0f, TURN_SPEED);
}

// ============== JOYSTICK CONTROL ==============

/**
 * Read joystick X axis (left-right)
 * Returns -100 to +100 (negative = left, positive = right)
 */
int readJoystickX() {
  int raw = analogRead(JOYSTICK_X_PIN);
  int centered = raw - JOYSTICK_CENTER;

  // Apply deadzone
  if (abs(centered) < JOYSTICK_DEADZONE) return 0;

  // Map to -100 to +100
  return map(centered, -512, 512, -100, 100);
}

/**
 * Read joystick Y axis (forward-backward)
 * Returns -100 to +100 (negative = backward, positive = forward)
 */
int readJoystickY() {
  int raw = analogRead(JOYSTICK_Y_PIN);
  int centered = raw - JOYSTICK_CENTER;

  // Apply deadzone
  if (abs(centered) < JOYSTICK_DEADZONE) return 0;

  // Map to -100 to +100 (inverted because Y axis is typically inverted)
  return map(centered, -512, 512, 100, -100);
}

/**
 * Handle joystick control mode
 * GUIDELINE 2: Direction and speed change based on joystick movement
 */
void handleJoystickControl() {
  int x = readJoystickX();
  int y = readJoystickY();

  // If joystick is in center (deadzone), stop motors
  if (x == 0 && y == 0) {
    motorStop();
    return;
  }

  // Calculate speed based on joystick magnitude
  float magnitude = sqrt(x*x + y*y);
  uint8_t speed = constrain((uint8_t)magnitude, 0, 100);

  // Determine direction
  if (abs(y) > abs(x)) {
    // Primarily forward/backward
    if (y > 0) {
      // Forward with slight turning
      uint8_t leftSpeed = speed;
      uint8_t rightSpeed = speed;

      if (x > 0) {
        // Turn right while moving forward
        rightSpeed = speed * (100 - abs(x)) / 100;
      } else if (x < 0) {
        // Turn left while moving forward
        leftSpeed = speed * (100 - abs(x)) / 100;
      }

      digitalWrite(MOTOR_L_DIR_PIN, MOTOR_FORWARD);
      digitalWrite(MOTOR_R_DIR_PIN, MOTOR_FORWARD);
      analogWrite(MOTOR_L_PWM_PIN, percentToPwm(leftSpeed));
      analogWrite(MOTOR_R_PWM_PIN, percentToPwm(rightSpeed));
    } else {
      // Backward with slight turning
      uint8_t leftSpeed = speed;
      uint8_t rightSpeed = speed;

      if (x > 0) {
        rightSpeed = speed * (100 - abs(x)) / 100;
      } else if (x < 0) {
        leftSpeed = speed * (100 - abs(x)) / 100;
      }

      digitalWrite(MOTOR_L_DIR_PIN, MOTOR_BACKWARD);
      digitalWrite(MOTOR_R_DIR_PIN, MOTOR_BACKWARD);
      analogWrite(MOTOR_L_PWM_PIN, percentToPwm(leftSpeed));
      analogWrite(MOTOR_R_PWM_PIN, percentToPwm(rightSpeed));
    }
  } else {
    // Primarily turning in place
    if (x > 0) {
      motorTurnRight(speed);
    } else {
      motorTurnLeft(speed);
    }
  }
}

// ============== ESP COMMAND HANDLING ==============

/**
 * Read and parse commands from ESP via Serial1
 * Commands: dist:<cm>, deg:<angle>, north, ping
 */
void readESPCommands() {
  while (Serial1.available() > 0) {
    String command = Serial1.readStringUntil('\n');
    command.trim();

    if (command.length() == 0) continue;

    // Heartbeat ping
    if (command == "ping") {
      lastHeartbeatTime = millis();
      heartbeatEnabled = true;
      Serial1.println("pong");
      continue;
    }

    // Distance command: dist:<cm> (positive = forward, negative = backward)
    if (command.startsWith("dist:")) {
      String numberPart = command.substring(5);
      targetDistance = numberPart.toFloat();
      commandType = "dist";
      hasNewCommand = true;
      lastESPCommand = command;
      Serial1.print("Received distance command: ");
      Serial1.println(targetDistance);
    }
    // Degree/heading command: deg:<angle>
    else if (command.startsWith("deg:")) {
      String numberPart = command.substring(4);
      targetHeading = numberPart.toFloat();

      // Normalize to 0-360
      while (targetHeading < 0) targetHeading += 360.0f;
      while (targetHeading >= 360) targetHeading -= 360.0f;

      commandType = "deg";
      hasNewCommand = true;
      lastESPCommand = command;
      Serial1.print("Received heading command: ");
      Serial1.println(targetHeading);
    }
    // Find North command
    else if (command == "north") {
      commandType = "north";
      hasNewCommand = true;
      lastESPCommand = "north";
      Serial1.println("Received Find North command");
    }
    else {
      Serial1.print("Unknown command: ");
      Serial1.println(command);
    }
  }
}

/**
 * Execute the pending ESP command
 */
void executeESPCommand() {
  detachInterrupt(digitalPinToInterrupt(JOYSTICK_BUTTON_PIN));
  if (commandType == "dist") {
    bool forward = (targetDistance >= 0);
    drive(abs(targetDistance), forward, DRIVE_SPEED);
  }
  else if (commandType == "deg") {
    turn(targetHeading, TURN_SPEED);
  }
  else if (commandType == "north") {
    findNorth();
  }

  delay(100);
  modeButtonPressed = false;
  attachInterrupt(digitalPinToInterrupt(JOYSTICK_BUTTON_PIN), modeButtonISR, FALLING);
  commandType = "";
}

// ============== LCD DISPLAY ==============

/**
 * Update LCD with current status
 * GUIDELINE 2: Display mode, compass, pulses, distances, joystick values, ESP commands
 */
void updateLCD() {
  lcd.clear();

  // Line 0: Mode and compass
  lcd.setCursor(0, 0);
  lcd.print(espMode ? "ESP " : "JOY ");

  if (compassConnected) {
    float heading = readCompassCalibrated();
    lcd.print((int)heading);
    lcd.print((char)223);  // Degree symbol
    lcd.print(getDirectionName(heading));
  } else {
    lcd.print("NO CMP");
  }

  // Line 1: Context-specific information
  lcd.setCursor(0, 1);

  if (espMode) {
    // ESP mode: Show last command
    lcd.print("C:");
    lcd.print(lastESPCommand.substring(0, 18));  // Truncate if needed
  } else {
    // Joystick mode: Show joystick values
    int x = readJoystickX();
    int y = readJoystickY();
    lcd.print("X:");
    lcd.print(x);
    lcd.print(" Y:");
    lcd.print(y);
  }

  // Line 2: Pulse counts for both wheels
  lcd.setCursor(0, 2);
  lcd.print("P L:");
  lcd.print(encoderLeftPulses);
  lcd.print(" R:");
  lcd.print(encoderRightPulses);

  // Line 3: Distance traveled for both wheels
  lcd.setCursor(0, 3);
  lcd.print("D L:");
  lcd.print((int)totalDistanceLeft);
  lcd.print(" R:");
  lcd.print((int)totalDistanceRight);
  lcd.print("cm");
}

/**
 * Display error message on LCD
 */
void displayError(String errorMsg) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ERROR:");
  lcd.setCursor(0, 1);
  lcd.print(errorMsg);
}

// ============== INTERRUPT SERVICE ROUTINES ==============

/**
 * Left encoder interrupt handler
 */
void encoderLeftISR() {
  encoderLeftPulses++;
}

/**
 * Right encoder interrupt handler
 */
void encoderRightISR() {
  encoderRightPulses++;
}

/**
 * Mode switch button interrupt handler
 * GUIDELINE 2: Switch between joystick and ESP-controlled mode
 */
void modeButtonISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastButtonTime > DEBOUNCE_DELAY) {
    modeButtonPressed = true;
    lastButtonTime = currentTime;
    Serial.println("BUTTON INTERRUPT");
  }
}
