#include <Wire.h>
#include "LiquidCrystal_I2C.h"

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Wire.beginOnPins(6, 5);
  // put your setup code here, to run once:
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  // initialize the LCD
  lcd.begin();
  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.print("Hello, world!");
}

void loop() {
  // put your main code here, to run repeatedly:

}
