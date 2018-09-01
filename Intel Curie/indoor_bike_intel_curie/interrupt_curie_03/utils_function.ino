void getNowTime() {
  Serial.print("Time now is: ");
  print2digits(hour());
  Serial.print(":");
  print2digits(minute());
  Serial.print(":");
  print2digits(second());

  Serial.print(" ");

  Serial.print(day());
  Serial.print("/");
  Serial.print(month());
  Serial.print("/");
  Serial.print(year());

  Serial.println();
}

void print2digits(int number) {
  if (number >= 0 && number < 10) {
    Serial.print('0');
  }
  Serial.print(number);
}
