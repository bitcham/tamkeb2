// Exercise 2++
#include <Arduino.h>

// Motor driver pinout (same as Exercise 1 for consistency).
const uint8_t MOTOR_FORWARD = 0;
const uint8_t MOTOR_BACKWARD = 1;
const uint8_t MOTOR_L_DIR_PIN = 7;
const uint8_t MOTOR_R_DIR_PIN = 8;
const uint8_t MOTOR_L_PWM_PIN = 9;
const uint8_t MOTOR_R_PWM_PIN = 10;

// Joystick pinout: two analog axes and one optional press button (not used here).
const uint8_t JOYSTICK_X_PIN = A8; // X axis controls turning (left/right)
const uint8_t JOYSTICK_Y_PIN = A9; // Y axis controls speed (forward/backward)

// Joystick calibration constants.
const int ANALOG_CENTER = 512;  // Ideal neutral position from analogRead()
const int ANALOG_RANGE = 512;   // Half-scale (0-1023 => -512..+511 around center)
const int DEADZONE = 40;        // Ignore small deviations around center to keep the car still

const uint8_t PWM_MAX = 255;
const uint8_t MAX_SPEED_PERCENT = 100; // Allow full speed; adjust lower if the car is too fast

struct MotorCommand {
  uint8_t leftDirection;
  uint8_t rightDirection;
  uint8_t leftPwm;
  uint8_t rightPwm;
};

MotorCommand computeMotorCommand(int rawX, int rawY);
int normalizeAxis(int reading);
uint8_t speedPercentToPwm(int percent);
void applyMotorCommand(const MotorCommand &command);

void setup() {
  pinMode(MOTOR_L_DIR_PIN, OUTPUT);
  pinMode(MOTOR_R_DIR_PIN, OUTPUT);
  pinMode(MOTOR_L_PWM_PIN, OUTPUT);
  pinMode(MOTOR_R_PWM_PIN, OUTPUT);

  stopMotors();
  Serial.begin(9600); // Helpful for tuning joystick response.
}

void loop() {
  int xReading = analogRead(JOYSTICK_X_PIN);
  int yReading = analogRead(JOYSTICK_Y_PIN);

  MotorCommand command = computeMotorCommand(xReading, yReading);
  applyMotorCommand(command);

  // Optional debug output; comment out if not needed during vehicle tuning.
  Serial.print("Joystick X: ");
  Serial.print(xReading);
  Serial.print(" Y: ");
  Serial.print(yReading);
  Serial.print(" | PWM L/R: ");
  Serial.print(command.leftPwm);
  Serial.print("/");
  Serial.println(command.rightPwm);

  delay(20); // Small delay to smooth out PWM updates without adding noticeable lag.
}

MotorCommand computeMotorCommand(int rawX, int rawY) {
  int turnPercent = normalizeAxis(rawX);
  int speedPercent = normalizeAxis(rawY);
  turnPercent = turnPercent * 2;

  // Масштабирование поворота: чем выше скорость, тем меньше чувствительность X
  int speedAbs = abs(speedPercent);
  int scale = map(100 - speedAbs, 0, 100, 30, 100); 
  // при высокой скорости (speedAbs≈100) поворот работает на 30%
  // при низкой скорости (speedAbs≈0) — на 100%

  turnPercent = (turnPercent * scale) / 100;

  // Differential mixing: adjust wheel speeds based on desired turn.
  int leftPercent = speedPercent + turnPercent;
  int rightPercent = speedPercent - turnPercent;

  leftPercent = constrain(leftPercent, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);
  rightPercent = constrain(rightPercent, -MAX_SPEED_PERCENT, MAX_SPEED_PERCENT);

  MotorCommand command;
  
  
  
  command.leftDirection  = (rightPercent >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;
  command.rightDirection = (leftPercent  >= 0) ? MOTOR_FORWARD : MOTOR_BACKWARD;

  command.leftPwm  = speedPercentToPwm(abs(rightPercent));
  command.rightPwm = speedPercentToPwm(abs(leftPercent));


  // If both joystick axes sit inside the deadzone, force a hard stop.
  if (command.leftPwm == 0 && command.rightPwm == 0) {
    command.leftDirection = MOTOR_FORWARD;
    command.rightDirection = MOTOR_FORWARD;
  }

  return command;
}

int normalizeAxis(int reading) {
  int offset = reading - ANALOG_CENTER;

  if (abs(offset) < DEADZONE) {
    return 0;
  }

  long scaled = (long)offset * 100 / ANALOG_RANGE; // Scale to approximately -100..100
  scaled = constrain(scaled, -100, 100);
  return (int)scaled;
}

uint8_t speedPercentToPwm(int percent) {
  percent = constrain(percent, 0, MAX_SPEED_PERCENT);
  long pwm = (long)percent * PWM_MAX / MAX_SPEED_PERCENT;
  return (uint8_t)pwm;
}

void applyMotorCommand(const MotorCommand &command) {
  digitalWrite(MOTOR_L_DIR_PIN, command.leftDirection);
  digitalWrite(MOTOR_R_DIR_PIN, command.rightDirection);
  analogWrite(MOTOR_L_PWM_PIN, command.leftPwm);
  analogWrite(MOTOR_R_PWM_PIN, command.rightPwm);
}

void stopMotors() {
  analogWrite(MOTOR_L_PWM_PIN, 0);
  analogWrite(MOTOR_R_PWM_PIN, 0);
}
