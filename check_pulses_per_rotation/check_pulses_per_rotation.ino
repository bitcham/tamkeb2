#include <Arduino.h>

const uint8_t ENCODER_PIN = 3;
const uint8_t ENCODER_INTERRUPT_MODE = CHANGE;

volatile unsigned long encoderPulses = 0;

void encoderISR() {
  encoderPulses++;
}

void setup() {
  pinMode(ENCODER_PIN, INPUT);
  Serial.begin(9600);
  
  Serial.println("*** СЧЕТЧИК ИМПУЛЬСОВ ЭНКОДЕРА ***");
  Serial.println("1. Поставьте метку на колесе");
  Serial.println("2. Медленно поворачивайте колесо рукой");
  Serial.println("3. Когда метка сделает полный оборот - запомните число");
  Serial.println("4. Для сброса счетчика введите 'r' и нажмите Enter");
  Serial.println("");
  
  encoderPulses = 0;
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN), encoderISR, ENCODER_INTERRUPT_MODE);
}

void loop() {
  // Показываем текущее количество импульсов каждые 200ms
  static unsigned long lastPrintTime = 0;
  static unsigned long lastPulseCount = 0;
  
  if (millis() - lastPrintTime >= 200) {
    unsigned long currentPulses = encoderPulses;
    
    // Печатаем только если количество изменилось
    if (currentPulses != lastPulseCount) {
      Serial.print("Импульсов: ");
      Serial.println(currentPulses);
      lastPulseCount = currentPulses;
    }
    
    lastPrintTime = millis();
  }
  
  // Проверяем команду сброса
  if (Serial.available()) {
    char command = Serial.read();
    if (command == 'r' || command == 'R') {
      encoderPulses = 0;
      Serial.println("*** СЧЕТЧИК СБРОШЕН ***");
    }
  }
}