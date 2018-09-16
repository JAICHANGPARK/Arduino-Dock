uint16_t startAddress = 0xd200;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  uint8_t topAddress = (startAddress >> 12) & 0x0f;
  Serial.print("topAddress -> "); Serial.println(topAddress, HEX);
  uint8_t topAddressDataCommand = (0x10 | topAddress);
  uint8_t top2Address = (startAddress >> 8) & 0x0f;
  Serial.print("top2Address ->"); Serial.println(top2Address, HEX);
  uint8_t top2AddressDataCommand =  (0x20 | top2Address);

  uint8_t bottomAddress = (uint8_t)((startAddress & 0x00f0) >> 4);
  uint8_t bottomAddressDataCommand = (0x10 | bottomAddress);
  uint8_t bottom2Address = (uint8_t)(startAddress & 0x000f);
  uint8_t bottom2AddressDataCommand = (0x20 | bottom2Address);

  Serial.println(topAddressDataCommand, HEX);
  Serial.println(top2AddressDataCommand, HEX);
  delay(1000);


  Serial.println(bottomAddressDataCommand, HEX);
  Serial.println(bottom2AddressDataCommand, HEX);
  delay(1000);


}
