/******************************************************************************
                               플레시 메모리 함수
                               @author: 박제창
                               1차 제작일 : 2018-09-11
 *********************************************************************************/
int getDeviceID() {
  uint8_t rcvByte[3];

  waitForWrite();
  writeEnable();

  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);
  SPI.transfer(COMMAND_READ_ID);
  rcvByte[0] = SPI.transfer((byte)0x00);
  rcvByte[1] = SPI.transfer((byte)0x00);
  rcvByte[2] = SPI.transfer((byte)0x00);
  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);

  return rcvByte[2];
}

void writeEnable() {
  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);
  SPI.transfer(COMMAND_WRITE_ENABLE);
  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);
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
  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);
  SPI.transfer(CMD_RDSR1);
  r1 = SPI.transfer(0xff);
  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);
  if (r1 & SR1_BUSY_MASK) {
    return true;
  }
  return false;
}

byte readStatusRegister(void) {
  byte result = 0;
  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);
  SPI.transfer(COMMAND_READ_STATUSREG);
  result = SPI.transfer((byte)0x00);
  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);
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

  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);

  SPI.transfer(sndByte[0]);
  SPI.transfer(sndByte[1]);
  SPI.transfer(sndByte[2]);
  SPI.transfer(sndByte[3]);

  rcvByte = SPI.transfer((byte)0x00);

  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);

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

  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);

  SPI.transfer(sndBytes[0]);
  SPI.transfer(sndBytes[1]);
  SPI.transfer(sndBytes[2]);
  SPI.transfer(sndBytes[3]);

  for (int i = 0; i < 256; i++) {
    SPI.transfer(writeData[i]);
  }
  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);
}


void writeInfoIndex(uint8_t* writeData, int addr) {
  uint8_t sndBytes[4] = { 0 };

  sndBytes[0] = COMMAND_PAGE_PROGRAM;
  sndBytes[1] = (uint16_t)(addr >> 16);
  sndBytes[2] = (uint16_t)(addr >> 8);
  sndBytes[3] = (uint16_t)addr;

  waitForWrite();
  writeEnable();

  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);

  SPI.transfer(sndBytes[0]);
  SPI.transfer(sndBytes[1]);
  SPI.transfer(sndBytes[2]);
  SPI.transfer(sndBytes[3]);

  for (int i = 0; i < 6; i++) {
    SPI.transfer(writeData[i]);
  }
  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);
}

/**
   운동 종료후 운동 정보를 저장하는 부분
*/
void writeFitnessData(uint8_t* writeData, int addr, int writeSize) {

  uint8_t sndBytes[4] = { 0 };
  //  int saveIndex = writeSize - 1;

  sndBytes[0] = COMMAND_PAGE_PROGRAM;
  sndBytes[1] = (uint16_t)(addr >> 16);
  sndBytes[2] = (uint16_t)(addr >> 8);
  sndBytes[3] = (uint16_t)addr;

  waitForWrite();
  writeEnable();

  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);

  SPI.transfer(sndBytes[0]);
  SPI.transfer(sndBytes[1]);
  SPI.transfer(sndBytes[2]);
  SPI.transfer(sndBytes[3]);

  for (int i = 0; i < writeSize; i++) {
    SPI.transfer(writeData[i]);
  }
  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);
}


void subSectorErase(int addr) {
  uint8_t sndBytes[4] = { 0 };

  sndBytes[0] = COMMAND_SUB_SECTOR_ERASE;
  sndBytes[1] = (addr >> 16);
  sndBytes[2] = (addr >> 8);
  sndBytes[3] = addr;

  waitForWrite();
  writeEnable();
  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);
  SPI.transfer(sndBytes[0]);
  SPI.transfer(sndBytes[1]);
  SPI.transfer(sndBytes[2]);
  SPI.transfer(sndBytes[3]);
  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);
  delay(100);
}

void sectorErase(int addr) {
  uint8_t sndBytes[4] = { 0 };

  sndBytes[0] = COMMAND_SECTOR_ERASE;
  sndBytes[1] = (addr >> 16);
  sndBytes[2] = (addr >> 8);
  sndBytes[3] = addr;

  waitForWrite();
  writeEnable();
  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);
  SPI.transfer(sndBytes[0]);
  SPI.transfer(sndBytes[1]);
  SPI.transfer(sndBytes[2]);
  SPI.transfer(sndBytes[3]);
  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);
  delay(100);
}

void allMemoryErase() {
  writeSR(0x00);
  waitForWrite();
  writeEnable();
  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);
  SPI.transfer(COMMAND_BULK_ERASE);
  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);
}

void writeSR(uint8_t setReg) {
  waitForWrite();
  writeEnable();
  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);
  SPI.transfer(COMMAND_WRITE_STATUSREG);
  SPI.transfer(setReg);
  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);
}

uint16_t N25Q256_read(uint32_t addr, uint8_t *buf, uint16_t n)
{
  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);

  SPI.transfer(CMD_READ_DATA);
  SPI.transfer(addr >> 16);
  SPI.transfer((addr >> 8) & 0xff);
  SPI.transfer(addr & 0xff);

  uint16_t i;
  for (i = 0; i < n; i++ ) {
    buf[i] = SPI.transfer(0x00);
  }

  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);
  return i;
}

uint16_t N25Q256_readFitness(uint32_t addr, uint8_t *buf, uint16_t n)
{
  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);

  SPI.transfer(CMD_READ_DATA);
  SPI.transfer(addr >> 16);
  SPI.transfer((addr >> 8) & 0xff);
  SPI.transfer(addr & 0xff);

  uint16_t i;
  for (i = 0; i < n; i++ ) {
    buf[i] = SPI.transfer(0x00);
  }

  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);
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

  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);

  SPI.transfer(CMD_PAGEPROG);
  SPI.transfer((addr >> 16) & 0xff);
  SPI.transfer((addr >> 8) & 0xff);
  SPI.transfer(addr & 0xff);

  for (i = 0; i < n; i++) {
    SPI.transfer(data[i]);
  }

  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);

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

  digitalWrite(FLASH_CHIP_SELECT_PIN, LOW);

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
  digitalWrite(FLASH_CHIP_SELECT_PIN, HIGH);

  return i;
}

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

void dump(uint8_t *dt, uint32_t n)
{

  uint32_t sz;
  char buf[64];
  uint16_t clm = 0;
  uint8_t data;
  uint8_t sum;
  uint8_t vsum[SAVE_UNIT_STEP];
  uint8_t total = 0;
  uint32_t saddr = 0;
  uint32_t eaddr = n - 1;
  sz = eaddr - saddr;

  Serial.print("----------------------------------------------------------\n");
  for (uint16_t i = 0; i < SAVE_UNIT_STEP; i++) {
    vsum[i] = 0;
  }
  for (uint32_t addr = saddr; addr <= eaddr; addr++) {
    data = dt[addr];
    if (clm == 0) {
      sum = 0;
      sprintf(buf, "%05lx: ", addr);
      Serial.print(buf);
    }

    sum += data; // sum = sum + data
    vsum[addr % SAVE_UNIT_STEP] += data;

    sprintf(buf, "%02x ", data); // 입력된 데이터 파라미터
    Serial.print(buf);

    clm++;

    if (clm == SAVE_UNIT_STEP) {
      sprintf(buf, "|%02x ", sum);
      Serial.print(buf);
      Serial.print("\n");
      clm = 0;
    }
  }
  Serial.print("----------------------------------------------------------\n");
  Serial.print("       ");
  for (uint16_t i = 0; i < SAVE_UNIT_STEP; i++) {
    total += vsum[i];
    sprintf(buf, "%02x ", vsum[i]);
    Serial.print(buf);
  }
  sprintf(buf, "|%02x ", total);
  Serial.print(buf);
  Serial.print("");
  Serial.print("");
  Serial.print("\n");
}
