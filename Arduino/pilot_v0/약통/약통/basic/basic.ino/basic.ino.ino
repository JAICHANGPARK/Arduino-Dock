#include "HX711.h"
HX711 scale(A2, A3); // DOUT, SCK

void setup()
{
  Serial.begin(9600);
  scale.set_scale(-523.5); //// this value is obtained by calibrating the scale with known weights
  scale.tare();
}

void loop()
{
  Serial.print("one reading:\t");
  Serial.print(scale.get_units(), 1);
  Serial.println(" g");

  delay(100);
}

