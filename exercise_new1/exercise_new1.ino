// Exercise 1: Variable speed distance driving with joystick button trigger
// 20cm at 50%, 5cm at 25%, 10cm at 75% (total 35cm)

#include <Arduino.h>
#include <LiquidCrystal.h>

// Motor control pins
const uint8_t MOTOR_FORWARD = 0;
const uint8_t MOTOR_BACKWARD = 1;
const uint8_t MOTOR_L_DIR_PIN = 7;
const uint8_t MOTOR_R_DIR_PIN = 8;
const uint8_t MOTOR_L_PWM_PIN = 9;
const uint8_t MOTOR_R_PWM_PIN = 10;

// Encoder pins
const uint8_t ENCODER_R_PIN = 2;  // Right encoder
const uint8_t ENCODER_L_PIN = 3;  // Left encoder
const uint8_t ENCODER_INTERRUPT_MODE = CHANGE;

// Joystick button pin (ISR supported pin)
const uint8_t JOYSTICK_BUTTON_PIN = 18;  // Pin 18 supports interrupts on Mega

// LCD pins
const int LCD_RS = 53;
const int LCD_E = 51;
const int LCD_D4 = 35;
const int LCD_D5 = 34;
const int LCD_D6 = 33;
const int LCD_D7 = 32;

// Drive parameters - distances in cm
const float SEGMENT1_DISTANCE_CM = 20.0f;  // 50% speed
const float SEGMENT2_DISTANCE_CM = 5.0f;   // 25% speed
const float SEGMENT3_DISTANCE_CM = 10.0f;  // 75% speed

const uint8_t SPEED_SEGMENT1 = 50;  // 50%
const uint8_t SPEED_SEGMENT2 = 25;  // 25%
const uint8_t SPEED_SEGMENT3 = 75;  // 75%

// Calibration value from previous exercises
const float PULSES_PER_CM = 83.3f; 
const float PULSES_PER_CM_L = 84.0755f;  // Adjust based on your measurements

// Encoder pulse counters
volatile unsigned long encoderRightPulses = 0;
volatile unsigned long encoderLeftPulses = 0;

// Button state
volatile bool buttonPressed = false;
volatile unsigned long lastButtonTime = 0;
const unsigned long DEBOUNCE_DELAY = 300;  // ms

// LCD object
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Function declarations
void encoderRightISR();
void encoderLeftISR();
void buttonISR();
void driveSequence();
void driveDistance(float distanceCm, uint8_t speedPercent);
void setMotorSpeed(uint8_t speedPercent);
void stopMotors();
uint8_t percentToPwm(uint8_t percent);
void updateLCD(float targetDist, float currentDist, uint8_t speed);

void setup() {
  // Initialize motor pins
  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_R_PWM_PIN, OUTPUT);

  // Initialize encoder pins
  pinMode(ENCODER_R_PIN, INPUT);
  pinMode(ENCODER_L_PIN, INPUT);

  // Initialize button pin
  pinMode(JOYSTICK_BUTTON_PIN, INPUT_PULLUP);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Exercise 1");
  lcd.setCursor(0, 1);
  lcd.print("Press button");

  // Initialize Serial for debugging
  Serial.begin(9600);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), encoderRightISR, ENCODER_INTERRUPT_MODE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), encoderLeftISR, ENCODER_INTERRUPT_MODE);
  attachInterrupt(digitalPinToInterrupt(JOYSTICK_BUTTON_PIN), buttonISR, FALLING);

  Serial.println("System ready. Press joystick button to start.");
}

void loop() {
  // Check if button was pressed
  if (buttonPressed) {
    buttonPressed = false;

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Starting...");
    delay(1000);

    // Execute the driving sequence
    driveSequence();

    // Display completion message
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("FINISHED!");
    lcd.setCursor(0, 1);
    lcd.print("Press for retry");

    Serial.println("=== Drive completed ===");
    Serial.println("Press button to run again.");
  }

  delay(50);
}

void driveSequence() {
  Serial.println("=== Starting drive sequence ===");

  // Segment 1: 20cm at 50% speed
  Serial.println("Segment 1: 20cm at 50%");
  driveDistance(SEGMENT1_DISTANCE_CM, SPEED_SEGMENT1);

  // Segment 2: 5cm at 25% speed
  Serial.println("Segment 2: 5cm at 25%");
  driveDistance(SEGMENT2_DISTANCE_CM, SPEED_SEGMENT2);

  // Segment 3: 10cm at 75% speed
  Serial.println("Segment 3: 10cm at 75%");
  driveDistance(SEGMENT3_DISTANCE_CM, SPEED_SEGMENT3);

  // Stop motors at the end
  stopMotors();
}

void driveDistance(float distanceCm, uint8_t speedPercent) {
  // Reset pulse counters
  encoderLeftPulses = 0;
  encoderRightPulses = 0;

  // Calculate target pulses
  unsigned long targetPulses = (unsigned long)(distanceCm * PULSES_PER_CM_L);

  // Set motor speed
  setMotorSpeed(speedPercent);

  // Drive until target distance reached
  while (encoderLeftPulses < targetPulses) {
    float currentDistance = encoderLeftPulses / PULSES_PER_CM_L;
    updateLCD(distanceCm, currentDistance, speedPercent);
    delay(50);
  }

  // Final distance reached
  float actualDistance = encoderLeftPulses / PULSES_PER_CM_L;
  Serial.print("Target: ");
  Serial.print(distanceCm);
  Serial.print("cm, Actual: ");
  Serial.print(actualDistance);
  Serial.println("cm");
}

void setMotorSpeed(uint8_t speedPercent) {
  digitalWrite(MOTOR_L_DIR_PIN, MOTOR_FORWARD);
  digitalWrite(MOTOR_R_DIR_PIN, MOTOR_FORWARD);
  uint8_t pwmValue = percentToPwm(speedPercent);
  analogWrite(MOTOR_L_PWM_PIN, pwmValue);
  analogWrite(MOTOR_R_PWM_PIN, pwmValue);
}

void stopMotors() {
  analogWrite(MOTOR_L_PWM_PIN, 0);
  analogWrite(MOTOR_R_PWM_PIN, 0);
}

uint8_t percentToPwm(uint8_t percent) {
  percent = constrain(percent, 0, 100);
  return (uint8_t)((percent * 255UL) / 100UL);
}

void updateLCD(float targetDist, float currentDist, uint8_t speed) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Spd:");
  lcd.print(speed);
  lcd.print("% Tgt:");
  lcd.print(targetDist, 0);

  lcd.setCursor(0, 1);
  lcd.print("Dist:");
  lcd.print(currentDist, 1);
  lcd.print("cm");
}

void encoderRightISR() {
  encoderRightPulses++;
}

void encoderLeftISR() {
  encoderLeftPulses++;
}

void buttonISR() {
  unsigned long currentTime = millis();
  if (currentTime - lastButtonTime > DEBOUNCE_DELAY) {
    buttonPressed = true;
    lastButtonTime = currentTime;
  }
}
