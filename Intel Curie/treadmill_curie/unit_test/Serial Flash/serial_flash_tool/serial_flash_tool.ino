/**

   CODE : JAICHANGPARK aka DREAMWALKER
   인텔 큐리 보드 사용
   시리얼 플레시 메모리 
   관리 툴 
   삭제 
   디버그
   데이터 확인 

*/

#include <CurieTime.h>
#include <CurieBLE.h>
#include <SPI.h>
#include "Crypto.h"

#define MAX_AES_PROCESS 32

#define DEBUG
#define ERGOMETER_LEXPA
#define USE_OLED


#define INFO_ADDRESS              0x000000
#define BASE_ADDRESS              0x001000
#define SAVE_UNIT_STEP            16      // 메모리 저장 스탭 크기
#define SAVE_FITNESS_BUFFER_SIZE  16      // 1단위 데이터 저장 버퍼 크기

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

#define CMD_READ_DATA             0x03
#define CMD_RDSR1                 0x05
#define SR1_BUSY_MASK             0x01
#define SR1_WEN_MASK              0x02
#define CMD_PAGEPROG              0x02

#define BLE_LED_INDICATOR_PIN         4

// 시리얼 플레시 메모리
const int chipSelectPin = 10;
int device_id, addr, data, i = 0;
uint8_t page[256];
uint8_t buf[256] = {0,};
uint8_t init_buff[16] = {0,};
uint8_t w_data[16] = {};
uint16_t n;

uint16_t register_index = 0;  // 저장 인덱스 총괄
uint32_t base_address = 0;
uint32_t now_address = 0;

volatile uint16_t uintDistanceKm = 0;
volatile uint32_t sumDistanceKm = 0x0000;  // 평균을 구하기 위한 이동거리 변수 ( 더 해짐)
volatile uint32_t sumSpeed = 0x0000;  // 평균 속도를 구하기 위한 속도 합 변수 (더 해짐)
volatile uint32_t sumHeartRate = 0;   // 평균 심박수를 구하기 위한 변수

//심박수 처리
uint8_t globalHeartRate = 0; // 심박수 전역 변수
uint16_t heartRateCount = 0; // 심박수 평균을 구하기위한 계수 카운터 변수
boolean heartRateMeasureFlag = false; // 심박 센서 착용여부 플레그
boolean heartRateLocationFlag = false; // 심박 센서 위치 확인플레그

boolean bleDateTimeSycnFlag = false; // 블루투스를 통해 시간 동기화가 되었을시 처리하는 플래그
boolean bleAuthCheckFlag = false; // 사용자 인증을 위한 플래그 실패시 false 성공시 true
boolean bleDataSyncFlag = false; // 데이터 전송 요청 이 들어왔을겨우 올바른 데이터 형식이면 true 아니면 false



/**

   프로그램 초기화 1회 실행 - 박제창

*/
void setup() {

  // put your setup code here, to run once:
#ifdef DEBUG
  Serial.begin(115200);
#endif

  pinMode(BLE_LED_INDICATOR_PIN, OUTPUT);

  digitalWrite(BLE_LED_INDICATOR_PIN, HIGH);
  delay(1000);
  digitalWrite(BLE_LED_INDICATOR_PIN, LOW);

  //시리얼 플레시 메모리 설정
  SPI.begin();
  pinMode(chipSelectPin, OUTPUT);

  writeEnable();
  init_buff[0] = normalRead(INFO_ADDRESS);
  init_buff[1] = normalRead(INFO_ADDRESS + 1);
  register_index = (((init_buff[0] << 8 ) & 0xff00) | (init_buff[1] & 0xff));
#ifdef DEBUG
  Serial.println(init_buff[0], HEX);
  Serial.println(init_buff[1], HEX);
  Serial.println(register_index, HEX);
#endif
  if (register_index == 0xffff) {
#ifdef DEBUG
    Serial.println("처음사용자입니다.");
#endif
    register_index = 0;
  } else {
#ifdef DEBUG
    Serial.print("기존 사용자입니다.");
    Serial.println(register_index);
#endif
    register_index = register_index + 1; // 저장된 인덱스를 읽어 왔기 떄문에
    //메모리의 데이터 파트에는 읽어온 데이터의 인덱스가 존재합니다.
    //기존 저장된 인덱스 보다 커야합니다.
  }

  dump(init_buff, 16);
  memset(buf, 0, 256);
  n = N25Q256_read(BASE_ADDRESS, buf, 256); // buf 메모리에 읽어온 256개의 데이터를 저장한다.
  dump(buf, 256); // 읽어온 데이터를 확인하는 함수

  Serial.print("d : "); Serial.println("메모리값 확인하기");
  Serial.print("e : "); Serial.println("메모리값 삭제하기");
  
}

void loop() {

}


/******************************************************************************
                                  플레시 메모리 함수
                      제작 : 박제창
 ***************************/
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


void writeInfoIndex(uint8_t* writeData, int addr) {
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

  for (int i = 0; i < 6; i++) {
    SPI.transfer(writeData[i]);
  }
  digitalWrite(chipSelectPin, HIGH);
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

  digitalWrite(chipSelectPin, LOW);

  SPI.transfer(sndBytes[0]);
  SPI.transfer(sndBytes[1]);
  SPI.transfer(sndBytes[2]);
  SPI.transfer(sndBytes[3]);

  for (int i = 0; i < writeSize; i++) {
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
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(sndBytes[0]);
  SPI.transfer(sndBytes[1]);
  SPI.transfer(sndBytes[2]);
  SPI.transfer(sndBytes[3]);
  digitalWrite(chipSelectPin, HIGH);
  delay(100);
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

uint16_t N25Q256_readFitness(uint32_t addr, uint8_t *buf, uint16_t n)
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

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    Serial.println(inChar);
    if (inChar == 'd') {
      Serial.println(inChar);
      memset(buf, 0, 256);
      writeEnable();
      n = N25Q256_read(BASE_ADDRESS, buf, 256); // buf 메모리에 읽어온 256개의 데이터를 저장한다.
      dump(buf, 256); // 읽어온 데이터를 확인하는 함수
    }

    if (inChar == 'e') {
      Serial.println(inChar);
      memset(buf, 0, 256);
      writeEnable();
      subSectorErase(INFO_ADDRESS);
      writeEnable();
      subSectorErase(BASE_ADDRESS);
      
      writeEnable();
      n = N25Q256_read(BASE_ADDRESS, buf, 256); // buf 메모리에 읽어온 256개의 데이터를 저장한다.
      dump(buf, 256); // 읽어온 데이터를 확인하는 함수
    }

    if (inChar == 'x') {
      Serial.println(inChar);
      memset(buf, 0, 256);
      writeEnable();
      allMemoryErase();
      writeEnable();
      n = N25Q256_read(BASE_ADDRESS, buf, 256); // buf 메모리에 읽어온 256개의 데이터를 저장한다.
      dump(buf, 256); // 읽어온 데이터를 확인하는 함수
    }


    if (inChar == 's') {

      //       memset(buf, 0, 256);
      for (int i = 0 ; i < register_index; i++) {
        uint8_t syncBuff[SAVE_FITNESS_BUFFER_SIZE]  = {0,};
        int tmpAddress = BASE_ADDRESS + (SAVE_UNIT_STEP * i);
        n = N25Q256_read(tmpAddress, syncBuff, SAVE_UNIT_STEP); // buf 메모리에 읽어온 16개 데이터를 저장한다.
        for (int k = 0 ; k < SAVE_UNIT_STEP; k ++) {
          Serial.print(syncBuff[k], HEX);
        }
        Serial.println("");
      }
    }
  }
}

