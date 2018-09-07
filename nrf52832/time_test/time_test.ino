void setup() {
  set_time(1256729737);  // Set RTC time to Wed, 28 Oct 2009 11:35:37
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {

  time_t seconds = time(NULL);

  Serial.print("Time as seconds since January 1, 1970 = %d\n");
  Serial.print(seconds);


  Serial.print("Time as a basic string = %s");
  Serial.println(ctime(&seconds));
  char buffers[32];
  strftime(buffers, 32, "%I:%M %p\n", localtime(&seconds));
  Serial.print("Time as a custom formatted string = %s");
  Serial.println(buffers);

  wait(1);
  // put your main code here, to run repeatedly:

}
