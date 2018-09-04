#include "Crypto.h"

#define MAX_AES_PROCESS 32

static byte aes_key[16] = {2, 2, 2, 2, 0, 0, 0, 0, 1, 1, 1, 1, 8, 8, 8, 8};
static byte aes_iv[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static int mode;
static byte aes_result[MAX_AES_PROCESS];

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  uint8_t key[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
  //  byte data[] = "123456789012345"; //16 chars == 16 bytes
  byte data[] = "9876543210000001"; //16 chars == 16 bytes
  Serial.println("Original Text");
  for (int i = 0; i < 16; i++) {
    Serial.print(data[i]);
    Serial.print(", ");
  }

  //  Serial.println(data);

  AES.Initialize(aes_iv, aes_key);
  byte encrypted[16]; // AHA! needs to be large, 2x is not enough
  byte aes_result[16];
  //  AES.Encrypt(key, 16, data , 0);
  Serial.println("Encrypt");
  AES.Encrypt(data, 16, encrypted, 0);
  for (int i = 0; i < 16; i++) {
    Serial.print(encrypted[i], HEX);
    Serial.print(", ");
  }
  Serial.println("Decrypt");
  AES.Decrypt(encrypted, 16, aes_result, 0);
  for (int i = 0; i < 16; i++) {
    Serial.print(aes_result[i]);
    Serial.print(", ");
  }

}

void loop() {
  //  Serial.println(data);
  // put your main code here, to run repeatedly:

  //  Serial.println("set Loop");


}
