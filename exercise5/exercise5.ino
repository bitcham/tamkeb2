// Exercise 5: Drive a set distance (e.g., 100 cm) forwards
// Print ENC A left and right pulse counts on LCD

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

// LCD pins (adjust according to your wiring)
const int LCD_RS = 53;
const int LCD_E = 51;
const int LCD_D4 = 35;
const int LCD_D5 = 34;
const int LCD_D6 = 33;
const int LCD_D7 = 32;

// Drive parameters
const float TARGET_DISTANCE_CM = 15.0f;  // Set your desired distance here
const uint8_t DRIVE_SPEED_PERCENT = 35;   // Same speed as Exercise 4

// From Exercise 4 measurements - adjust these values based on your actual measurements
const float PULSES_PER_CM = 83.3f;  // Replace with your measured value from Exercise 4
const float PULSES_PER_CM_L = 84.0755f; //27.2 before gear down
const unsigned long UPDATE_INTERVAL_MS = 100;  // LCD update interval

// Encoder pulse counters
volatile unsigned long encoderRightPulses = 0;
volatile unsigned long encoderLeftPulses = 0;

// LCD object
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Function declarations
void encoderRightISR();
void encoderLeftISR();
void startForwardDrive();
void stopMotors();
uint8_t percentToPwm(uint8_t percent);
void updateLCD();
float calculateDistance(unsigned long pulses);

void setup() {
  // Initialize motor pins
  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_R_PWM_PIN, OUTPUT);
  
  // Initialize encoder pins
  pinMode(ENCODER_R_PIN, INPUT);
  pinMode(ENCODER_L_PIN, INPUT);  
  
  // Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Starting...");
  
  // Initialize Serial for debugging
  Serial.begin(9600);
  
  // Reset pulse counters
  encoderRightPulses = 0;
  encoderLeftPulses = 0;
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN), encoderRightISR, ENCODER_INTERRUPT_MODE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN), encoderLeftISR, ENCODER_INTERRUPT_MODE); 
  
  delay(2000);  // Brief pause before starting
  
  // Calculate target pulse count
  unsigned long targetPulses = (unsigned long)(TARGET_DISTANCE_CM * PULSES_PER_CM_L);
  
  Serial.println("--Starting distance drive...--");
  Serial.print("Target distance: ");
  Serial.print(TARGET_DISTANCE_CM);
  Serial.println(" cm");
  Serial.print("Target pulses: ");
  Serial.println(targetPulses);
  
  // Start driving
  startForwardDrive();
  
  unsigned long lastUpdateTime = millis();
  
  // Drive until target distance is reached
  while (encoderLeftPulses < targetPulses) {
    // Update LCD periodically
    if (millis() - lastUpdateTime >= UPDATE_INTERVAL_MS) {
      updateLCD();
      lastUpdateTime = millis();
    }
    
    // Small delay to prevent overwhelming the system
    // delay(10);
  }
  
  // Stop motors
  stopMotors();
  
  // Detach interrupts
  detachInterrupt(digitalPinToInterrupt(ENCODER_R_PIN));
  detachInterrupt(digitalPinToInterrupt(ENCODER_L_PIN)); 
  
  // Final measurements
  unsigned long finalRightPulses = encoderRightPulses;
  unsigned long finalLeftPulses = encoderLeftPulses;
  float actualDistance = calculateDistance(finalLeftPulses);
  
  // Display final results
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("FINISHED!");
  lcd.setCursor(0, 1);
  lcd.print("Dist: ");
  lcd.print(actualDistance, 1);
  lcd.print("cm");
  
  // Serial output for debugging
  Serial.println("=== DRIVE COMPLETED ===");
  Serial.print("Target distance: ");
  Serial.print(TARGET_DISTANCE_CM);
  Serial.println(" cm");
  Serial.print("Actual distance: ");
  Serial.print(actualDistance);
  Serial.println(" cm");
  Serial.print("Right encoder pulses: ");
  Serial.println(finalRightPulses);
  Serial.print("Left encoder pulses: ");
  Serial.println(finalLeftPulses);
  
  delay(3000);
  
  // Show final pulse counts on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("R:");
  lcd.print(finalRightPulses);
  lcd.print(" L:");
  lcd.print(finalLeftPulses);
}

void loop() {
  // Empty - all work done in setup()
}

void encoderRightISR() {
  encoderRightPulses++;
}

void encoderLeftISR() {
  encoderLeftPulses++;
}

void startForwardDrive() {
  digitalWrite(MOTOR_L_DIR_PIN, MOTOR_FORWARD);
  digitalWrite(MOTOR_R_DIR_PIN, MOTOR_FORWARD);
  uint8_t pwmValue = percentToPwm(DRIVE_SPEED_PERCENT);
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

void updateLCD() {
  float currentDistance = calculateDistance(encoderLeftPulses);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("R:");
  lcd.print(encoderRightPulses);
  lcd.print(" L:");
  lcd.print(encoderLeftPulses);
  
  lcd.setCursor(0, 1);
  lcd.print("Dist:");
  lcd.print(currentDistance, 1);
  lcd.print("cm");
}

float calculateDistance(unsigned long pulses) {
  if (PULSES_PER_CM_L > 0.0f) {
    return pulses / PULSES_PER_CM_L;
  }
  return 0.0f;
}