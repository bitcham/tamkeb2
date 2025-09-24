#include <Arduino.h>


  
const int ENCA = 2;
volatile long encoderCount = 0;

void setup() {

  pinMode(ENCA, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  Serial.begin(9600);
}

void loop() {
  Serial.println(encoderCount);
}

void readEncoder() {
  encoderCount++;
}