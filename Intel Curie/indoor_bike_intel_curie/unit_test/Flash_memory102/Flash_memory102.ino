#include <SPI.h>

#define COMMAND_WRITE_ENABLE      (byte) 0x06
#define COMMAND_RANDOM_READ       (byte)0x03
#define COMMAND_FAST_READ         (byte)0x0B
#define COMMAND_PAGE_PROGRAM      (byte)0x02
#define COMMAND_SECTOR_ERASE      (byte) 0xD8
#define COMMAND_BULK_ERASE        (byte) 0xC7
#define COMMAND_READ_STATUSREG    (uint8_t)0x05
#define COMMAND_READ_ID           (byte)0x9E
#define COMMAND_WRITE_STATUSREG   (uint8_t)0x01
#define COMMAND_READ_FLAG_STATUS  0x70
#define COMMAND_SUB_SECTOR_ERASE  0x20

#define CMD_READ_DATA           0x03
#define CMD_RDSR1               0x05
#define SR1_BUSY_MASK           0x01
#define SR1_WEN_MASK            0x02
#define CMD_PAGEPROG            0x02

#define BASE_ADDRESS            0x000000

const int chipSelectPin = 10;
int id, addr, data, i = 0;
void setup() {

  uint8_t page[256];

  Serial.begin(9600); // initialize Serial communication
  while (!Serial) ;   // wait for serial port to connect.
  // start the SPI library:
  SPI.begin();
  uint8_t buf[256] = {0,};
  uint8_t w_data[16] = {};
  uint16_t n;           // 取得データ数


  pinMode(chipSelectPin, OUTPUT);

  //  allMemoryErase();
  //  // put your setup code here, to run once:
  id = getDeviceID();
  Serial.print("id: "); Serial.println(id, HEX);
  //  //  allMemoryErase();
  //
  subSectorErase(BASE_ADDRESS);
  memset(buf, 0, 256);
  Serial.print("W25Q64_eraseSector : ");
  Serial.println(n);
  n =  N25Q256_read(0, buf, 256);
  dump(buf, 256);

  for (uint8_t i = 0; i < 16; i++) {
    w_data[i] = 'A' + i;
    Serial.print(w_data[i], HEX);
  }
  Serial.println("");

  memset(buf, 0, 256); // buf 시작 주소부터 256개를 모두 0으로 세트한다.
  n = writePage2(w_data, 0x00, 16);
  Serial.print("page Write(0,10,d,16byte) : n = ");
  Serial.println(n);
  n = N25Q256_read(0, buf, 256); // buf 메모리에 읽어온 256개의 데이터를 저장한다.
  dump(buf, 256); // 읽어온 데이터를 확인하는 함수

  //populate array
  for (int i = 0; i < 256; i++) {
    page[i] = i;
  }

  //  writeEnable();
  //  writePage(page, BASE_ADDRESS);
  //
  //  Serial.print("sr check after write: " ); Serial.println(readStatusRegister());
  //  n =  N25Q256_read(0, buf, 256);
  //  dump(buf, 256);
  //
  //  //  for (int i = 0; i < 256; i++) {
  //  //    Serial.print(buf[i], HEX);
  //  //  }


}

void loop() {
  // put your main code here, to run repeatedly:

}

int getDeviceID() {
  uint8_t rcvByte[3];

  waitForWrite();
  writeEnable();

  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(COMMAND_READ_ID);
  rcvByte[0] = SPI.transfer((byte)0x00);
  rcvByte[1] = SPI.transfer((byte)0x00);
  rcvByte[2] = SPI.transfer((byte)0x00);
  digitalWrite(chipSelectPin, HIGH);

  return rcvByte[2];
}

void writeEnable() {
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(COMMAND_WRITE_ENABLE);
  digitalWrite(chipSelectPin, HIGH);
}

void waitForWrite() {
  byte result = 0;
  result = readStatusRegister();
  while (result & 0x1) {
    result = readStatusRegister();
  }
}

bool N25Q256_IsBusy()
{
  uint8_t r1;
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(CMD_RDSR1);
  r1 = SPI.transfer(0xff);
  digitalWrite(chipSelectPin, HIGH);
  if (r1 & SR1_BUSY_MASK) {
    return true;
  }
  return false;
}

byte readStatusRegister(void) {
  byte result = 0;
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(COMMAND_READ_STATUSREG);
  result = SPI.transfer((byte)0x00);
  digitalWrite(chipSelectPin, HIGH);
  return result;
}

//prints data at a given address
void printData (unsigned int addr)
{
  data = normalRead(addr);
  Serial.print("Address 0x");
  Serial.print(addr, HEX);
  Serial.print(": 0x");
  Serial.println(data, HEX);
}

uint8_t normalRead(int addr) {
  uint8_t sndByte[4] = { 0 };
  uint8_t rcvByte;

  sndByte[0] = COMMAND_RANDOM_READ;

  sndByte[1] = (addr >> 16);
  sndByte[2] = (addr >> 8);
  sndByte[3] = addr;

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

void writePage(uint8_t* writeData, int addr) {
  uint8_t sndBytes[4] = { 0 };

  sndBytes[0] = COMMAND_PAGE_PROGRAM;
  sndBytes[1] = (uint16_t)(addr >> 16);
  sndBytes[2] = (uint16_t)(addr >> 8);
  sndBytes[3] = (uint16_t)addr;

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

void subSectorErase(int addr) {
  uint8_t sndBytes[4] = { 0 };

  sndBytes[0] = COMMAND_SUB_SECTOR_ERASE;
  sndBytes[1] = (addr >> 16);
  sndBytes[2] = (addr >> 8);
  sndBytes[3] = addr;

  waitForWrite();
  writeEnable();
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(sndBytes[0]);
  SPI.transfer(sndBytes[1]);
  SPI.transfer(sndBytes[2]);
  SPI.transfer(sndBytes[3]);
  digitalWrite(chipSelectPin, HIGH);
  delay(10);
}

void allMemoryErase() {
  writeSR(0x00);
  waitForWrite();
  writeEnable();
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(COMMAND_BULK_ERASE);
  digitalWrite(chipSelectPin, HIGH);
}

void writeSR(uint8_t setReg) {
  waitForWrite();
  writeEnable();
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(COMMAND_WRITE_STATUSREG);
  SPI.transfer(setReg);
  digitalWrite(chipSelectPin, HIGH);
}

uint16_t N25Q256_read(uint32_t addr, uint8_t *buf, uint16_t n)
{
  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(CMD_READ_DATA);
  SPI.transfer(addr >> 16);
  SPI.transfer((addr >> 8) & 0xff);
  SPI.transfer(addr & 0xff);

  uint16_t i;
  for (i = 0; i < n; i++ ) {
    buf[i] = SPI.transfer(0x00);
  }

  digitalWrite(chipSelectPin, HIGH);
  return i;
}

uint16_t N25Q256_pageWrite(uint16_t sect_no, uint16_t inaddr, uint8_t* data, uint8_t n)
{

  uint32_t addr = sect_no;
  int i;
  addr <<= 12;
  addr += inaddr;
  waitForWrite();
  writeEnable();
  //
  //  if (N25Q256_IsBusy()) {
  //    return 0;
  //  }

  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(CMD_PAGEPROG);
  SPI.transfer((addr >> 16) & 0xff);
  SPI.transfer((addr >> 8) & 0xff);
  SPI.transfer(addr & 0xff);

  for (i = 0; i < n; i++) {
    SPI.transfer(data[i]);
  }

  digitalWrite(chipSelectPin, HIGH);

  waitForWrite();

  return i;
}

uint16_t writePage2(uint8_t* writeData, int addr, uint8_t n) {
  uint8_t sndBytes[4] = { 0 };

  sndBytes[0] = COMMAND_PAGE_PROGRAM;
  sndBytes[1] = (uint16_t)(addr >> 16);
  sndBytes[2] = (uint16_t)(addr >> 8);
  sndBytes[3] = (uint16_t)addr;

  waitForWrite();
  writeEnable();

  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(sndBytes[0]);
  SPI.transfer(sndBytes[1]);
  SPI.transfer(sndBytes[2]);
  SPI.transfer(sndBytes[3]);

  //  for (int i = 0; i < 256; i++) {
  //    SPI.transfer(writeData[i]);
  //  }

  for (i = 0; i < n; i++) {
    SPI.transfer(writeData[i]);
  }
  digitalWrite(chipSelectPin, HIGH);

  return i;
}





