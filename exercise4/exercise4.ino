// Exercise 4: drive forward for 4 seconds while counting encoder pulses with an ISR.
#include <Arduino.h>

const uint8_t MOTOR_FORWARD = 1;
const uint8_t MOTOR_BACKWARD = 0;
const uint8_t MOTOR_L_DIR_PIN = 7;
const uint8_t MOTOR_R_DIR_PIN = 8;
const uint8_t MOTOR_L_PWM_PIN = 9;
const uint8_t MOTOR_R_PWM_PIN = 10;

const uint8_t ENCODER_PIN = 3;              // Attach encoder A signal (left or right) here
const uint8_t ENCODER_INTERRUPT_MODE = CHANGE; // Count rising edges; change if using quadrature

const unsigned long DRIVE_DURATION_MS = 4000;
const uint8_t DRIVE_SPEED_PERCENT = 35;     // Adjust to a speed that feels safe while testing

// Wheel diameter: 4.2cm, Circumference = π × 4.2 = 13.19cm
const float WHEEL_CIRCUMFERENCE_CM = 13.19f;
const float PULSES_PER_REV = 372.0f;  // Adjust based on your encoder

volatile unsigned long encoderPulses = 0;

void encoderISR();
void startForwardDrive();
void stopMotors();
uint8_t percentToPwm(uint8_t percent);

void setup() {
  delay(2000);
  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_R_PWM_PIN, OUTPUT);
  pinMode(ENCODER_PIN, INPUT);

  Serial.begin(9600);

  encoderPulses = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, ENCODER_INTERRUPT_MODE);

  startForwardDrive();
  unsigned long startTime = millis();
  while (millis() - startTime < DRIVE_DURATION_MS) {
    // Busy-wait for the duration; adjust if you want to monitor something else.
  }

  stopMotors();
  detachInterrupt(digitalPinToInterrupt(ENCODER_PIN));

  unsigned long pulses = encoderPulses;


  float distanceCm = 0.0f;
  if (PULSES_PER_REV > 0.0f) {
    distanceCm = (pulses / PULSES_PER_REV) * WHEEL_CIRCUMFERENCE_CM;
  }

  float pulsesPerCm = (distanceCm > 0.0f) ? (pulses / distanceCm) : 0.0f;

  Serial.println("Measurement summary:");
  Serial.print("Pulses counted: ");
  Serial.println(pulses);
  Serial.print("Estimated distance (cm): ");
  Serial.println(distanceCm, 2);
  Serial.print("Pulses per cm: ");
  Serial.println(pulsesPerCm, 2);
  Serial.println("Repeat the test on the table and floor to compare values.");
}
void loop() {
}

void encoderISR() {
  encoderPulses++;
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
