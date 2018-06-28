int sensorPin = 1;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  int HumVal = analogRead(5);
  int TempVal = analogRead(4);

  float HumVoltage = (HumVal / 1024.0) * 3.3;
  float TempVoltage = (TempVal / 1024.0) * 3.3;
  Serial.print("HumVoltage");
  Serial.println(HumVoltage);
  Serial.print("TempVoltage");
  Serial.println(HumVoltage);
  float Hum = -12.5 + (125 * (HumVoltage / 3.3));
  float Temp = -66.875 + (218.75 * (TempVoltage / 3.3));
  float FTemp = -88.375 + (393.75 * (TempVoltage / 3.3));

  Serial.println(Hum);
  Serial.println(Temp);
  delay(1000);

}
