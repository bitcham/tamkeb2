#include <LiquidCrystal.h>

#define Motor_forward         0
#define Motor_return          1
#define Motor_L_dir_pin       7
#define Motor_R_dir_pin       8
#define Motor_L_pwm_pin       9
#define Motor_R_pwm_pin       10

const int LCD_RS = 53;
const int LCD_E = 51;
const int LCD_D4 = 35;
const int LCD_D5 = 34;
const int LCD_D6 = 33;
const int LCD_D7 = 32;

const int JOYSTICK_X = A8;  // X-axis analog pin
const int JOYSTICK_Y = A9;  // Y-axis analog pin
const int JOYSTICK_KEY = 19;  // Button digital pin

// Initialize LCD
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Variables for joystick values
int xValue = 0;
int yValue = 0;
int xPercent = 0;
int yPercent = 0;

// ISR variables
volatile bool runMotorTest = false;
volatile bool displayMode = false;  // false = analog data, true = button count
volatile unsigned long buttonPressCount = 0;
volatile unsigned long lastInterruptTime = 0;

// ISR function for joystick button press
void buttonISR() {
  unsigned long interruptTime = millis();
  
  // Debounce: ignore interrupt if it occurs within 200ms of the last one
  if (interruptTime - lastInterruptTime > 200) {
    runMotorTest = true;        
    buttonPressCount++;         
    displayMode = !displayMode; 
    lastInterruptTime = interruptTime;
  }
}

void setup() {
  Serial.begin(9600);
  
  // Initialize LCD (16 columns, 2 rows)
  lcd.begin(16, 2);
  lcd.clear();

  // Configure joystick pins
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
  pinMode(JOYSTICK_KEY, INPUT_PULLUP);
  
  // Configure motor pins
  pinMode(Motor_L_dir_pin, OUTPUT);
  pinMode(Motor_R_dir_pin, OUTPUT);
  pinMode(Motor_L_pwm_pin, OUTPUT);
  pinMode(Motor_R_pwm_pin, OUTPUT);
  
  // Attach interrupt for joystick button (pin 2 = interrupt 0)
  attachInterrupt(digitalPinToInterrupt(JOYSTICK_KEY), buttonISR, FALLING);
}

void loop() {
  // Read analog values from joystick (0-1023)
  xValue = analogRead(JOYSTICK_X);
  yValue = analogRead(JOYSTICK_Y);
  
  // Calculate percentages (0-100%)
  xPercent = map(xValue, 0, 1023, 0, 100);
  yPercent = map(yValue, 0, 1023, 0, 100);
  
  // Check if motor test should run
  if (runMotorTest) {
    runMotorTest = false;  // Reset flag
    motorTest();           // Execute motor test
  }
  
  
  // Display on LCD
  displayValues();
  
  // Send to Serial Monitor for debugging
  printToSerial();
  
  // Small delay to prevent flickering
  delay(500);
}

void motorTest() {
  int pwm_R = 0;
  int pwm_L = 0;
  
  for(int i = 99; i > 0; i--) {
    // Direction control
    digitalWrite(Motor_R_dir_pin, Motor_return);  
    digitalWrite(Motor_L_dir_pin, Motor_return);
    delay(2000);
    digitalWrite(Motor_R_dir_pin, Motor_forward);  
    digitalWrite(Motor_L_dir_pin, Motor_forward); 
    delay(2000);
    
    // PWM control
    pwm_R=0;
    pwm_L=0; 
    pwm_L = i;
    pwm_R = i;
    analogWrite(Motor_L_pwm_pin, pwm_L);
    analogWrite(Motor_R_pwm_pin, pwm_R);
  }
}

void displayValues() {
  lcd.clear();
  
  if (!displayMode) {
    // Display analog data (Exercise 1 functionality)
    lcd.setCursor(0, 0);
    lcd.print("X:");
    
    // Print analog value with padding for alignment
    if (xValue < 10) {
      lcd.print("   ");
    } else if (xValue < 100) {
      lcd.print("  ");
    } else if (xValue < 1000) {
      lcd.print(" ");
    }
    lcd.print(xValue);   
    
    lcd.print("  ");
    
    // Print percentage with padding
    if (xPercent < 10) {
      lcd.print(" ");
    }
    lcd.print(xPercent);
    lcd.print("%");
    
    // Line 2: Y values
    lcd.setCursor(0, 1);
    lcd.print("Y:");
    
    // Print analog value with padding for alignment
    if (yValue < 10) {
      lcd.print("   ");
    } else if (yValue < 100) {
      lcd.print("  ");
    } else if (yValue < 1000) {
      lcd.print(" ");
    }
    lcd.print(yValue);
    
    lcd.print("  ");
    
    // Print percentage with padding
    if (yPercent < 10) {
      lcd.print(" ");
    }
    lcd.print(yPercent);
    lcd.print("%");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Button Presses:");
    lcd.setCursor(0, 1);
    lcd.print("Count: ");
    lcd.print(buttonPressCount);
  }
}

void printToSerial() {
  Serial.print("X: ");
  Serial.print(xValue);
  Serial.print(" (");
  Serial.print(xPercent);
  Serial.print("%) | Y: ");
  Serial.print(yValue);
  Serial.print(" (");
  Serial.print(yPercent);
  Serial.print("%) | Mode: ");
  Serial.print(displayMode ? "Button Count" : "Analog");
  Serial.print(" | Count: ");
  Serial.println(buttonPressCount);
}