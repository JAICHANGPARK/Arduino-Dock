#include <Wire.h>
void setup() {
  Serial.begin(9600);
  Serial.println("heart rate sensor:");
  Wire.begin();
}
void loop() {
//  Serial.println("heart rate sensor:");
  Wire.requestFrom(0xA0 >> 1, 1);    // request 1 bytes from slave device
   unsigned char c = Wire.read();   // receive heart rate value (a byte)
    Serial.println(c, DEC); 

  delay(500);
}
