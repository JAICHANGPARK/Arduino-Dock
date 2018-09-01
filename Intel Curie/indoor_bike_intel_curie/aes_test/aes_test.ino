#include <Crypto.h>


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  uint8_t key[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
  char data[] = "0123456789012345"; //16 chars == 16 bytes
  Serial.println(data);
}

void loop() {
  // put your main code here, to run repeatedly:

}
