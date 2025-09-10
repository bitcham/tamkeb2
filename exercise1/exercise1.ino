

#include <LiquidCrystal.h>

const int LCD_RS = 53;
const int LCD_E = 51;
const int LCD_D4 = 35;
const int LCD_D5 = 34;
const int LCD_D6 = 33;
const int LCD_D7 = 32;

const int JOYSTICK_X = A8;  // X-axis analog pin
const int JOYSTICK_Y = A9;  // Y-axis analog pin

// Initialize LCD
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// Variables for joystick values
int xValue = 0;
int yValue = 0;
int xPercent = 0;
int yPercent = 0;

void setup() {
  Serial.begin(9600);
  
  // Initialize LCD (16 columns, 2 rows)
  lcd.begin(16, 2);
  lcd.clear();

  // Configure joystick pins
  pinMode(JOYSTICK_X, INPUT);
  pinMode(JOYSTICK_Y, INPUT);
}

void loop() {
  // Read analog values from joystick (0-1023)
  xValue = analogRead(JOYSTICK_X);
  yValue = analogRead(JOYSTICK_Y);
  
  // Calculate percentages (0-100%)
  xPercent = map(xValue, 0, 1023, 0, 100);
  yPercent = map(yValue, 0, 1023, 0, 100);
  
  // Clear LCD before updating
  
  // Display on LCD
  displayValues();
  
  // Send to Serial Monitor for debugging
  printToSerial();
  
  // Small delay to prevent flickering
  delay(100);
}

void displayValues() {
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
  lcd.print("%  ");
  
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
  lcd.print("%  ");
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
  Serial.println("%)");
}