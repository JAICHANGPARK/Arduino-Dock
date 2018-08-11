#include <SPI.h>


#define COMMAND_WRITE_ENABLE (byte) 0x06
#define COMMAND_RANDOM_READ (byte)0x03
#define COMMAND_FAST_READ (byte)0x0B
#define COMMAND_PAGE_PROGRAM (byte)0x02
#define COMMAND_SECTOR_ERASE (byte) 0xD8
#define COMMAND_BULK_ERASE (byte) 0xC7
#define COMMAND_READ_STATUSREG (uint8_t)0x05
#define COMMAND_READ_ID (byte)0x9E
#define COMMAND_WRITE_STATUSREG (uint8_t)0x01
#define COMMAND_READ_FLAG_STATUS 0x70
#define COMMAND_SUB_SECTOR_ERASE 0x20

#define BASE_ADDRESS 0x004000

const int chipSelectPin = 8;
uint8_t read_id[3];

int data;

void setup() {

  Serial.begin(9600); // initialize Serial communication
  while (!Serial);    // wait for serial port to connect.

  SPI.begin();
  // put your setup code here, to run once:
  pinMode(chipSelectPin, OUTPUT);
  Serial.print("begin --->");
  getDeviceIDV1();
  Serial.print("Manufacturer ID  "); Serial.println(read_id[0], HEX);
  Serial.print("Memory Type: "); Serial.println(read_id[1], HEX);
  Serial.print("Memory Capacity: "); Serial.println(read_id[2], HEX);

  for (int i = 0; i < 16; i++) {
    printData(BASE_ADDRESS + i);
  }

  uint8_t page[256];
  //populate array
  for (int i = 256; i < 1; i--) {
    page[i] = i;
  }

  writeEnable();
  writePage(page, BASE_ADDRESS);

  Serial.print("sr check after write: " ); Serial.println(readSR());

  for (int i = 0; i < 256; i++) {
    printData(BASE_ADDRESS + i);
  }

}

void loop() {
  // put your main code here, to run repeatedly:

}
void getDeviceIDV1() {
  waitForWrite();
  writeEnable();
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(COMMAND_READ_ID);
  read_id[0] = SPI.transfer((byte)0x00);
  read_id[1] = SPI.transfer((byte)0x00);
  read_id[2] = SPI.transfer((byte)0x00);
  digitalWrite(chipSelectPin, HIGH);


}

uint8_t* getDeviceID() {
  uint8_t rcvByte[3];
  waitForWrite();
  writeEnable();
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(COMMAND_READ_ID);
  rcvByte[0] = SPI.transfer((byte)0x00);
  rcvByte[1] = SPI.transfer((byte)0x00);
  rcvByte[2] = SPI.transfer((byte)0x00);

  digitalWrite(chipSelectPin, HIGH);

  return rcvByte;
}

byte readSR(void) {
  byte sr;
  unsigned int result = 0;
  //  mySPI.setSelect(LOW);
  digitalWrite(chipSelectPin, LOW);
  result = SPI.transfer(COMMAND_READ_STATUSREG); // command read status register
  sr = SPI.transfer(0x00);
  digitalWrite(chipSelectPin, HIGH);
  return sr;
}

void waitForWrite() {
  byte ss;
  ss = readSR();
  while (SR & 0x1)
    ss = readSR();
}

void writeEnable() {
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(COMMAND_WRITE_ENABLE);

  digitalWrite(chipSelectPin, HIGH);
}

//prints data at a given address
void printData (unsigned int address)
{
  data = normalRead(address);
  Serial.print("Address 0x");
  Serial.print(address, HEX);
  Serial.print(": 0x");
  Serial.println(data, HEX);
}

uint8_t normalRead(int address) {
  uint8_t sndByte[4] = { 0 };
  uint8_t rcvByte;

  sndByte[0] = COMMAND_RANDOM_READ;

  sndByte[1] = (address >> 16);
  sndByte[2] = (address >> 8);
  sndByte[3] = address;


  waitForWrite();
  writeEnable();

  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(sndByte[0]);
  SPI.transfer(sndByte[1]);
  SPI.transfer(sndByte[2]);
  SPI.transfer(sndByte[3]);

  rcvByte = SPI.transfer((byte)0x00);

  digitalWrite(chipSelectPin, HIGH);

  return rcvByte;
}

void writePage(uint8_t* writeData, int address) {
  uint8_t sndBytes[4] = { 0 };

  sndBytes[0] = COMMAND_PAGE_PROGRAM;
  sndBytes[1] = (uint16_t)(address >> 16);
  sndBytes[2] = (uint16_t)(address >> 8);
  sndBytes[3] = (uint16_t)address;

  waitForWrite();
  writeEnable();

  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(sndBytes[0]);
  SPI.transfer(sndBytes[1]);
  SPI.transfer(sndBytes[2]);
  SPI.transfer(sndBytes[3]);

  for (int i = 0; i < 256; i++) {
    SPI.transfer(writeData[i]);
  }

  digitalWrite(chipSelectPin, HIGH);
}
