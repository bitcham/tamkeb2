#include <Arduino.h>

const uint8_t MOTOR_FORWARD = 0;   
const uint8_t MOTOR_BACKWARD = 1; 
const uint8_t MOTOR_L_DIR_PIN = 7;   // Left motor direction pin
const uint8_t MOTOR_R_DIR_PIN = 8;   // Right motor direction pin
const uint8_t MOTOR_L_PWM_PIN = 9;   // Left motor PWM pin
const uint8_t MOTOR_R_PWM_PIN = 10;  // Right motor PWM pin


const uint8_t PWM_MAX = 255;
const uint8_t SPEED_30_PERCENT = (PWM_MAX * 30 + 50) / 100; // Rounded 30% duty cycle
const uint8_t SPEED_75_PERCENT = (PWM_MAX * 75 + 50) / 100; // Rounded 75% duty cycle

const unsigned long LEFT_FORWARD_DURATION_MS = 2000;   // 2 seconds forward on left wheel
const unsigned long RIGHT_FORWARD_DURATION_MS = 10000; // 10 seconds forward on right wheel
const unsigned long BACKWARD_DURATION_MS = 4000;       // 4 seconds backward on both wheels


void runMotorSequence();
void setMotorDirection(uint8_t leftDirection, uint8_t rightDirection);
void setMotorSpeeds(uint8_t leftSpeed, uint8_t rightSpeed);
void stopMotors();

void setup() {

  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_R_PWM_PIN, OUTPUT);

  stopMotors();       
  runMotorSequence(); 
  stopMotors();        
}
void loop() {

}

void runMotorSequence() {
  // Step 1: drive only the left wheel forward at about 30% speed for 2 seconds.
  setMotorDirection(MOTOR_FORWARD, MOTOR_FORWARD);
  setMotorSpeeds(SPEED_30_PERCENT, 0);
  delay(LEFT_FORWARD_DURATION_MS);

  // Step 2: drive only the right wheel forward at about 30% speed for 10 seconds.
  setMotorDirection(MOTOR_FORWARD, MOTOR_FORWARD);
  setMotorSpeeds(0, SPEED_30_PERCENT);
  delay(RIGHT_FORWARD_DURATION_MS);

  // Step 3: drive both wheels backward at about 75% speed for 4 seconds.
  setMotorDirection(MOTOR_BACKWARD, MOTOR_BACKWARD);
  setMotorSpeeds(SPEED_75_PERCENT, SPEED_75_PERCENT);
  delay(BACKWARD_DURATION_MS);
}

void setMotorDirection(uint8_t leftDirection, uint8_t rightDirection) {
  // Write the desired direction levels to the motor driver input pins.
  digitalWrite(MOTOR_L_DIR_PIN, leftDirection);
  digitalWrite(MOTOR_R_DIR_PIN, rightDirection);
}

void setMotorSpeeds(uint8_t leftSpeed, uint8_t rightSpeed) {
  // Apply PWM duty cycles to control the motor speeds.
  analogWrite(MOTOR_L_PWM_PIN, leftSpeed);
  analogWrite(MOTOR_R_PWM_PIN, rightSpeed);
}

void stopMotors() {
  // Force both PWM channels to zero to hold the car stationary.
  setMotorSpeeds(0, 0);
}
