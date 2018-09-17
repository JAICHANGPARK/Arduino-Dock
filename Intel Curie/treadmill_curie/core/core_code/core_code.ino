/*
      made by : JAICHANGPARK aka DREAMWALKER
      인텔 큐리 보드 사용
      32비트 쿼크 코어
      트레드밀 (KNU TM0)
*/

#include <CurieTime.h>
#include <CurieBLE.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include "Crypto.h"

#define DEBUG
#define TREADMILL
#define INDOOR_BIKE



#define TREADMILL_DISTANCE            0.13F         //단위 m 
#define WORKOUT_DONE_TIME_MILLIS      500           // 운동 자동 종료 시간 500ms후 모든 정보 초기화 및 변수 저장.
#define REAL_TIME_SEND_INTERVAL       1000          // 실시간 운동 시 1초 간격으로 데이터를 보내도록 .
#define ONE_SECOND                    1000
#define ONE_MINUTE                    (60 *1000)
#define CAL_MINUTE(X)                 (X * ONE_MINUTE)

#define MAX_AES_PROCESS               32

#define HR_ADDR                       0xA0    //심박 센서 주소 

#define INFO_ADDRESS                  0x00000000  // 기본 정보 저장 메모리 시작 주소
#define BASE_ADDRESS                  0x00001000  // 운동 정보 저장 메모리 주소 (종료 시 저장되는 곳)
#define DETAIL_ADDRESS                0x00170000  // 상세 운동 정보 저장 메모리 시작 주소 

#define SAVE_UNIT_STEP                22      // 메모리 저장 스탭 크기
#define SAVE_FITNESS_BUFFER_SIZE      22      // 1단위 데이터 저장 버퍼 크기

#define COMMAND_WRITE_ENABLE          (byte)0x06
#define COMMAND_RANDOM_READ           (byte)0x03
#define COMMAND_FAST_READ             (byte)0x0B
#define COMMAND_PAGE_PROGRAM          (byte)0x02
#define COMMAND_SECTOR_ERASE          (byte)0xD8
#define COMMAND_BULK_ERASE            (byte)0xC7
#define COMMAND_READ_STATUSREG        (uint8_t)0x05
#define COMMAND_READ_ID               (byte)0x9E
#define COMMAND_WRITE_STATUSREG       (uint8_t)0x01
#define COMMAND_READ_FLAG_STATUS      0x70
#define COMMAND_SUB_SECTOR_ERASE      0x20
#define CMD_READ_DATA                 0x03
#define CMD_RDSR1                     0x05
#define SR1_BUSY_MASK                 0x01
#define SR1_WEN_MASK                  0x02
#define CMD_PAGEPROG                  0x02

/*******************************************************
   Pin Mecro
 *******************************************************/
#define MAGNET_INTERRUPT_PIN          2
#define BLE_LED_INDICATOR_PIN         3
#define DIGITAL_PIN_RESERVED          4
#define RFID_RST_PIN                  5
#define RFID_SS_PIN                   6
#define TFT_DC                        7
#define TFT_RST                       8
#define TFT_CS                        9
#define FLASH_CHIP_SELECT_PIN         10


volatile uint32_t count = 0;          // 인터럽트 클럭 카운트 수
volatile float distance = 0.0f;       // 거리 변수 [ m]
volatile float distanceUnitKm = 0.0f; // 거리 변수 [km]
volatile float speedNow = 0.0f;       // 순간 속도 변수 [km/h]
volatile uint16_t uintSpeedNow = 0;
volatile uint32_t uintTotalDistance = 0;

volatile float roundSpeed = 0.0f;    // 반올림한 속도 변수 저장 .

/*************** 평균 운동량 정보 처리 변수    ****************************/
volatile uint16_t uintDistanceKm = 0;
volatile uint32_t sumDistanceKm = 0x0000;  // 평균을 구하기 위한 이동거리 변수 ( 더 해짐)
volatile uint32_t sumSpeed = 0x0000;  // 평균 속도를 구하기 위한 속도 합 변수 (더 해짐)
volatile uint32_t sumHeartRate = 0;   // 평균 심박수를 구하기 위한 변수

/*************** 심박수 처리 변수   ****************************/
uint8_t globalHeartRate = 0; // 심박수 전역 변수
uint16_t heartRateCount = 0; // 심박수 평균을 구하기위한 계수 카운터 변수

/*************** 프로그램 시간 관리 변수   ****************************/
volatile long t = 0;                  // 센서 입력 외부 인터럽트 시간 변수 : 운동 종료시 초기화 필요
volatile float InstantTime = 0.0f;    // 운동 종료시 초기화 필요
volatile int diffTime = 0.0f;         // 인터럽트 발생 시간 차를 구하기 위한 변수 :millisecond의 시간차를 확인한다.
volatile long startFitnessTime = 0;   // 운동 시작 시간 저장 변수 ( 1회 저장된다) 운동종료시 초기화 필요
volatile long endFitnessTime = 0;     // 운동 종료 시간 변수  : 운동 종료시 초기화 필요
volatile long workoutTime = 0;        // 운동 시간 계산 변수  : 운동 종료시 초기화 필요

long tftTimeIndex = 0;                // 디스플레이 변경을 위한 시간 변수
long saveMinTime = 0;                 // 임의 시간 인터럽트 해제를 위한 시간 변수 (카운트 계수에 사용된다)
long realTimePreviousMillis = 0;  // 실시간운동 정보 초기화를 위한 한계 시간 약 3초 이상 인터럽트가 없으면 초기화한다.
long rfidContactedTime = 0;

/*************** 프로그램 스위치 플래그  ****************************/
// 1byte를 가지고 마스크 처리해도 되긴한데 그냥 이렇게 하도록 한다.
// 시간이 읍다 흑

boolean deviceConnectedFlag = false;      //장비가 블루투스 연결이 되었는지 확인하는 플래그
boolean fitnessStartOrEndFlag = false;    // 운동의 시작과 종료를 판단하는 플레그
boolean heartRateMeasureFlag = false;     // 심박 센서 착용여부 플레그
boolean heartRateLocationFlag = false;    // 심박 센서 위치 확인플레그
boolean userRFIDCheckFlag = false;        // RFID 접촉 여부 확인 플레그

boolean bleDateTimeSycnFlag = false;      // 블루투스를 통해 시간 동기화가 되었을시 처리하는 플래그
boolean bleAuthCheckFlag = false;         // 사용자 인증을 위한 플래그 실패시 false 성공시 true
boolean bleDataSyncFlag = false;          // 데이터 전송 요청 이 들어왔을겨우 올바른 데이터 형식이면 true 아니면 false

/*****암호화 패킷 버퍼 ****************************/
static byte aes_key[16] = {2, 2, 2, 2, 0, 0, 0, 0, 1, 1, 1, 1, 8, 8, 8, 8};
static byte aes_iv[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static byte aes_result[MAX_AES_PROCESS];
byte authCoreValue[] = "9876543210100001"; //16 chars == 16 bytes
byte encrypted[16]; // AHA! needs to be large, 2x is not enough

/*****블루투스 패킷 버퍼 ****************************/
uint8_t heartRateData[] = {0b00000010, 0x00};
uint8_t treadmillData[] = {0x05, 0x00, 0x00, 0x00, 0x00};
uint8_t indoorBikeData[] = {0x00, 0x00, 0x00, 0x00};
uint8_t authData[] = {0x00, 0x00, 0x00};     // 결과 값 반환 패킷 데이터 버퍼
uint8_t resultPacket[] = {0x00, 0x00, 0x00}; // 결과 값 반환 패킷 데이터 버퍼

/*****시리얼 플레시 메모리 ****************************/
int device_id, addr, data, i = 0;
uint8_t page[256] = {0,};
uint8_t buf[256]  = {0,};
uint8_t init_buff[16] = {0,};
uint8_t w_data[16] = {0,};
uint16_t n = 0;
uint16_t register_index = 0;  // 저장 인덱스 총괄
uint32_t base_address = 0;
uint32_t now_address = 0;

/*****RFID 객체 변수 ****************************/
MFRC522 rfid(RFID_SS_PIN, RFID_RST_PIN); // Instance of the class
MFRC522::MIFARE_Key key;
byte nuidPICC[4] = {0xff, 0xff, 0xff, 0xff};

/****TFT 디스플레이 객체 **/
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);
int lcdCnt = 0;

/******전역 함수 정의 *******************************/
void printHex(byte *buffer, byte bufferSize);
void printDec(byte *buffer, byte bufferSize);
void interrupt_func();

extern BLECharacteristic indorBikeChar;
extern BLECharacteristic treadmillChar;
extern BLECharacteristic heartRateMeansurement;


void setup() {

#ifdef DEBUG
  Serial.begin(115200);
#endif
  Wire.begin();
  SPI.begin(); // Init SPI bus
  rfid.PCD_Init(); // Init MFRC522
  AES.Initialize(aes_iv, aes_key);  // aes 암호화 초기화
  attachInterrupt(MAGNET_INTERRUPT_PIN, interrupt_func, FALLING);
  pinMode(BLE_LED_INDICATOR_PIN, OUTPUT);
  digitalWrite(BLE_LED_INDICATOR_PIN, HIGH);
  delay(1000);
  digitalWrite(BLE_LED_INDICATOR_PIN, LOW);
  initTFTDisplay();
  setupBluetoothLowEnergy();

  pinMode(FLASH_CHIP_SELECT_PIN, OUTPUT);
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
    register_index = register_index + 1;  // 저장된 인덱스를 읽어 왔기 떄문에
    //메모리의 데이터 파트에는 읽어온 데이터의 인덱스가 존재합니다.
    //기존 저장된 인덱스 보다 커야합니다.
  }

  dump(init_buff, 16);

  memset(buf, 0, 256);
  n = N25Q256_read(BASE_ADDRESS, buf, 256); // buf 메모리에 읽어온 256개의 데이터를 저장한다.
  dump(buf, 256); // 읽어온 데이터를 확인하는 함수

}

void loop() {
  BLE.poll();
  long currentTimeIndicatorMillis = millis();

  if (deviceConnectedFlag) {      // 블루투스 연결은 되어 있는 상태
    if (fitnessStartOrEndFlag) { // 블루투스 연결은 되어 있는 상태에서 운동 중일 때만 실시간 전송이 되도록
      long currentMillis = millis();
      // if 200ms have passed, check the battery level:
      if (currentMillis - realTimePreviousMillis >= REAL_TIME_SEND_INTERVAL) {
        realTimePreviousMillis = currentMillis;
        indoorBikeData[2] = (uint8_t)(uintSpeedNow & 0xFF);
        indoorBikeData[3] = (uint8_t)(uintSpeedNow >> 8) & 0xFF;
        indorBikeChar.setValue(indoorBikeData, 4);

        treadmillData[2] = (uint8_t)(uintTotalDistance & 0xFF);
        treadmillData[3] = (uint8_t)(uintTotalDistance >> 8) & 0xFF;
        treadmillData[4] = (uint8_t)(uintTotalDistance >> 16) & 0xFF;
        treadmillChar.setValue(treadmillData, 5);
        updateHeartRateLevel();

#ifdef DEBUG
        Serial.println("블루투스 연결은 되어 있으면서 운동중입니다.");
        Serial.print("count -> "); Serial.print(count);
        Serial.print("| distance -> "); Serial.print(distance);
        Serial.print("| distanceUnitKm -> "); Serial.print(distanceUnitKm);
        Serial.print("| diffTime -> "); Serial.print(diffTime);
        Serial.print("| InstantTime -> "); Serial.print(InstantTime);
        Serial.print("| speedNow -> "); Serial.println(speedNow);
#endif

      }

      displaySet(&currentTimeIndicatorMillis); // 디스플레이 설정

    } else {
      //블루투스 연결은 되어 있지만 운동중이지 않을때 { == 운동 종료 상태일때)
#ifdef DEBUG
      Serial.println("블루투스 연결은 되어 있지만 운동 중이지 않습니다.");
#endif

    }
  }
  else {  //블루투스 연결되어 있지 않은 상태이면
    if (fitnessStartOrEndFlag) { //블루투스 연결되어 있지 않은 상태에서 운동 중이라면
      if (currentTimeIndicatorMillis - t >= WORKOUT_DONE_TIME_MILLIS) {//500 ms 이상 인터럽트 발생 없다면 운동  종료 처리 !

        /****************************************
           운동 종료 데이터 저장 처리
         ***************************************/
        uint16_t saveYear = year();   //년
        uint8_t saveMonth = month();  //월
        uint8_t saveDay = day();      //일
        uint8_t saveHour = hour();    //시
        uint8_t saveMin = minute();   //분
        uint8_t saveSecond = second(); // 초
        uint16_t meanSpeed = sumSpeed / count; //카운트 수를 기준으로 평균 운동 기록 연산
        int tmpTime = (int)(workoutTime / 1000.0f); // ms --> s 로 변환
        uint16_t saveWorkoutTime = (uint16_t)tmpTime; //운동 시간 변수
        // 이동 거리는 --> uintDistanceKm 변수 사용

        uint8_t meanHeartRate =  (uint8_t) (sumHeartRate / heartRateCount);  // 평균 심박수 연산
        uint32_t saveAddress = BASE_ADDRESS + (SAVE_UNIT_STEP * register_index); // 저장 주소 연산
        uint16_t meanKcal = 0;    //소모 열량 변수 defualt : 0

        writeEnable();       //서브섹터 삭제하기
        subSectorErase(INFO_ADDRESS); //서브섹터 삭제하기
#ifdef DEBUG
        Serial.print("sr check: " ); Serial.println(readStatusRegister());
        Serial.println("after the subsector is erased");
#endif

        int buffCount = 0;
        uint8_t save_info_index[6];  // 인덱스 저장을 위한 버퍼
        uint8_t fitnessDataBuff[SAVE_FITNESS_BUFFER_SIZE]; // 운동정보 저장을 위한 버퍼
        save_info_index[buffCount++] = (uint8_t)((register_index >> 8) & 0xff);
        save_info_index[buffCount++] = (uint8_t)(register_index & 0xff);
        save_info_index[buffCount++] =  (uint8_t)((saveAddress >> 24) & 0xff);
        save_info_index[buffCount++] =  (uint8_t)((saveAddress >> 16) & 0xff);
        save_info_index[buffCount++] =  (uint8_t)((saveAddress >> 8) & 0xff);
        save_info_index[buffCount++] = (uint8_t)(saveAddress & 0xff);
        writeEnable();
        writeInfoIndex(save_info_index, INFO_ADDRESS);
        buffCount = 0;

        fitnessDataBuff[buffCount++] = (uint8_t)((register_index >> 8) & 0xff);
        fitnessDataBuff[buffCount++] = (uint8_t)(register_index & 0xff);
        fitnessDataBuff[buffCount++] = nuidPICC[0];
        fitnessDataBuff[buffCount++] = nuidPICC[1];
        fitnessDataBuff[buffCount++] = nuidPICC[2];
        fitnessDataBuff[buffCount++] = nuidPICC[3];
        fitnessDataBuff[buffCount++] = (uint8_t)((saveYear >> 8) & 0xff);
        fitnessDataBuff[buffCount++] = (uint8_t)(saveYear & 0xff);
        fitnessDataBuff[buffCount++] = saveMonth & 0xff;
        fitnessDataBuff[buffCount++] = saveDay & 0xff;
        fitnessDataBuff[buffCount++] = saveHour & 0xff;
        fitnessDataBuff[buffCount++] = saveMin & 0xff;
        fitnessDataBuff[buffCount++] = saveSecond & 0xff;
        fitnessDataBuff[buffCount++] = (uint8_t)((saveWorkoutTime >> 8) & 0xff);
        fitnessDataBuff[buffCount++] = (uint8_t)(saveWorkoutTime & 0xff);
        fitnessDataBuff[buffCount++] = (uint8_t)((meanSpeed >> 8) & 0xff);
        fitnessDataBuff[buffCount++] = (uint8_t)(meanSpeed & 0xff);
        fitnessDataBuff[buffCount++] = (uint8_t)((uintDistanceKm >> 8) & 0xff);
        fitnessDataBuff[buffCount++] = (uint8_t)(uintDistanceKm & 0xff);
        fitnessDataBuff[buffCount++] = meanHeartRate & 0xff;
        fitnessDataBuff[buffCount++] = (uint8_t)((meanKcal >> 8) & 0xff);
        fitnessDataBuff[buffCount++] = (uint8_t)(meanKcal & 0xff);

        writeEnable();  // 운동량 정보 쓰기
        writeFitnessData(fitnessDataBuff, saveAddress, SAVE_UNIT_STEP); // 운동량 정보 쓰기

#ifdef DEBUG
        Serial.print("저장 주소 --> ");  Serial.print("0x"); Serial.println(saveAddress, HEX);
        Serial.print("인덱스 --> "); Serial.print((uint8_t)((register_index >> 8) & 0xff), HEX); Serial.println((uint8_t)(register_index & 0xff), HEX); //2바이트
        Serial.print("저장 시간 --> "); Serial.print((uint8_t)((saveYear >> 8) & 0xff), HEX); Serial.println((uint8_t)(saveYear & 0xff), HEX); //2바이트
        Serial.print("사용자 정보 태그 --> ");
        for (int i = 0; i < 4 ; i ++) { //사용자 태그 정보 초기화
          Serial.print(nuidPICC[i],HEX);
        }
        Serial.println("");
        Serial.print(saveMonth, HEX); //1바이트
        Serial.print(saveDay, HEX);   //1바이트
        Serial.print(saveHour, HEX);  //1바이트
        Serial.print(saveMin, HEX);   //1바이트
        Serial.println(saveSecond, HEX); //1바이트
        Serial.print("운동시간 --> "); Serial.println(saveWorkoutTime); // 2바이트
        Serial.print("평균 속도 --> "); Serial.println(meanSpeed); // 2바이트
        Serial.print("총 운동 거리 --> "); Serial.println(uintDistanceKm);  // 2바이트
        Serial.print("평균 심박수 --> "); Serial.println(meanHeartRate);  //1바이트
        Serial.print("소모 열량 --> "); Serial.println(0); //2바이트
#endif
        register_index++;
        buffCount = 0;

        //시스템변수 초기화
        count = 0;
        t = 0;
        distance = 0;
        distanceUnitKm = 0;
        startFitnessTime = 0;
        endFitnessTime = 0;
        sumSpeed = 0;
        workoutTime = 0;
        uintDistanceKm = 0;
        sumDistanceKm = 0x0000;
        sumSpeed = 0x0000;
        sumHeartRate = 0;

        for (int i = 0; i < 4 ; i ++) { //사용자 태그 정보 초기화
          nuidPICC[i] = 0xff;
        }

        // 시스템 플레그 초기화
        fitnessStartOrEndFlag = false;  //운동 종료 스위치
        userRFIDCheckFlag = false;

#ifdef DEBUG
        Serial.println("운동종료");
#endif
        // 시스템 디스플레이 종료 안내
        workoutDoneDisplay();// 운동 종료 표시해주기
        delay(3000);
        initTFTDisplay(); // 디스플레이 초기화

      } else { //블루투스 연결되어 있지 않지만 운동중입니다.
#ifdef DEBUG
        Serial.println("블루투스 연결되어 있지 않지만 운동중입니다.");
        Serial.print("count -> "); Serial.print(count);
        Serial.print("| distance -> "); Serial.print(distance);
        Serial.print("| distanceUnitKm -> "); Serial.print(distanceUnitKm);
        Serial.print("| diffTime -> "); Serial.print(diffTime);
        Serial.print("| InstantTime -> "); Serial.print(InstantTime);
        Serial.print("| speedNow -> "); Serial.print(speedNow);
        Serial.print("| roundSpeed -> "); Serial.println(roundSpeed);
#endif
      }

      unsigned char hr_realtime = readHeartRate(HR_ADDR);
      if (hr_realtime == 0) {
        heartRateMeasureFlag = false;
        heartRateLocationFlag = false;
      } else if (hr_realtime > 0 && hr_realtime < 60) {
        heartRateMeasureFlag = true;
        heartRateLocationFlag = false;
      } else {
        heartRateMeasureFlag = true;
        heartRateLocationFlag = true;
        globalHeartRate = hr_realtime;
        // 심박수 카운트 업 , 심박수 평균을 위한 더하기 수행 --> 운동 종료시 초기화 될 변수
        heartRateCount++;
        sumHeartRate += globalHeartRate;
      }
      delay(500);
      displaySet(&currentTimeIndicatorMillis); // 디스플레이 설정
    } else { //블루투스 연결되어 있지 않은 상태에서 운동 중이지 않다면, (== 운동중이지 않을때)
#ifdef DEBUG
      device_id = getDeviceID();
      getNowTime();
      unsigned char hr_test = readHeartRate(HR_ADDR);

      Serial.println("블루투스 연결되어 있지 않은 상태이고 운동 중이지 않습니다.");
      Serial.print("register_index: "); Serial.println(register_index, HEX);
      Serial.print("id: "); Serial.println(device_id, HEX);
      Serial.print("hr_first_check: "); Serial.println(hr_test, DEC);

      if (hr_test == 0) {
        heartRateMeasureFlag = false;
        heartRateLocationFlag = false;
        Serial.println("심박 센서 착용 안됨 : error code e400");
      } else if (hr_test > 0 && hr_test < 60) {
        heartRateMeasureFlag = true;
        heartRateLocationFlag = false;
        Serial.println("올바른 위치 확인 :  에러코드 e401");
      } else {
        heartRateMeasureFlag = true;
        heartRateLocationFlag = true;
        Serial.print("심박수 조건 완료 --> ");
        Serial.print("hr: "); Serial.println(hr_test, DEC);
        globalHeartRate = hr_test;
        // 심박수 카운트 업 , 심박수 평균을 위한 더하기 수행
      }
#endif
      delay(1000);
    }
  }

  //  if (currentTimeIndicatorMillis - saveMinTime >= CAL_MINUTE(10) ) {
  //    detachInterrupt(MAGNET_INTERRUPT_PIN);
  //    saveMinTime = 0;
  //  } else {
  //
  //  }

  rfid_address_read(); // RFID 아이디를 읽는다.
  //  initTFTDisplay(); // 비용이 많이 든다면 삭제하자.

  checkTFTDisplay(&currentTimeIndicatorMillis);

}

void rfid_address_read() {
  // Look for new cards
  if ( ! rfid.PICC_IsNewCardPresent()) {
    return;
  }
  // Verify if the NUID has been readed
  if ( ! rfid.PICC_ReadCardSerial()) {
    return;
  }

  Serial.print(F("PICC type: "));
  MFRC522::PICC_Type piccType = rfid.PICC_GetType(rfid.uid.sak);
  Serial.println(rfid.PICC_GetTypeName(piccType));

  // Check is the PICC of Classic MIFARE type
  if (piccType != MFRC522::PICC_TYPE_MIFARE_MINI &&
      piccType != MFRC522::PICC_TYPE_MIFARE_1K &&
      piccType != MFRC522::PICC_TYPE_MIFARE_4K) {
    Serial.println(F("Your tag is not of type MIFARE Classic."));
    return;
  }

  if (rfid.uid.uidByte[0] != nuidPICC[0] ||
      rfid.uid.uidByte[1] != nuidPICC[1] ||
      rfid.uid.uidByte[2] != nuidPICC[2] ||
      rfid.uid.uidByte[3] != nuidPICC[3] ) { // 읽어온 데이터와 기본 저장된 배열 매칭 작업

    // Store NUID into nuidPICC array
    for (byte i = 0; i < 4; i++) {
      nuidPICC[i] = rfid.uid.uidByte[i];
    }

    userRFIDCheckFlag = true;
    rfidContactedTime = millis(); // 테그 접촉 시간 저장


#ifdef DEBUG
    Serial.println(F("A new card has been detected."));
    Serial.println(F("The NUID tag is:"));
    Serial.print(F("In hex: "));
    printHex(rfid.uid.uidByte, rfid.uid.size);
    Serial.println();
    Serial.print(F("In dec: "));
    printDec(rfid.uid.uidByte, rfid.uid.size);
    Serial.println();
#endif
  }
  else {
    userRFIDCheckFlag = true;
    rfidContactedTime = millis(); // 이미 접촉된 태그여도 접촉된 시간은 저장되야함. 이 변수의 역할이 그럼
    Serial.println(F("Card read previously."));
  }

  rfid.PICC_HaltA();   // Halt PICC
  rfid.PCD_StopCrypto1();   // Stop encryption on PCD

  if (!fitnessStartOrEndFlag) { //블루투스연결은 상관 없이 운동중이지만 않을때 초기 디스플레이를 보여줘야 한다.
    initTFTDisplay();
    // 사용자가 태그를 찍었지만 운동을 하지 않을 경우를 생각해야함 .
    // 즉 태그만 찍고 운동을 시작하지 않고 그냥 내겨간 경우
  }
}

void checkTFTDisplay(long * currerntMillis) {
  if (*currerntMillis - rfidContactedTime >= 60000) { // 1분이 지나면
    if (userRFIDCheckFlag) { // 사용자 태그가 인식된 상태이면
      if (!fitnessStartOrEndFlag) { // 사용자 태그가 인식된 상태이지만 운동중이지 않을때
        userRFIDCheckFlag = false; // 초기화
        initTFTDisplay();
      }
    }
  }
}

void initTFTDisplay() {
  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black ta
  tft.fillScreen(ST77XX_BLACK);
  tft.invertDisplay(true);
  tft.setTextWrap(true);
  tft.setCursor(10, 10);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.println("Hello");

  String message = "";
  if (userRFIDCheckFlag) {
    for (int i = 0; i < 4; i++) {
      message +=  nuidPICC[i];
    }
  } else { // 인식되어 있지 않다면
    message = "Please contact your Tag before start workout";
  }

  tft.setCursor(0, 30);
  tft.setTextSize(2);
  tft.println(message);
}

void  workoutDoneDisplay() {
  tft.setTextWrap(true);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(5, 10);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.println("Autometic Save");
  String message = "Workout Done!";
  tft.setCursor(5, 50);
  tft.setTextSize(2);
  tft.println(message);
}

void displaySet(long * currerntMillis) {
  if (*currerntMillis - tftTimeIndex >= 3000) {
    int lcdModulo = lcdCnt % 5 ; // modulo 연산을 통해 표시 디스플레이 변경
    switch (lcdModulo) {
      case 0:
        // 사용자 태그 확인 디스플레이
        //        tftPrintTest3();
        tftPrintCheckUser();
        break;
      case 1:
        // 현재 운동 속도 디스플레이
        //        tftPrintTest4();
        tftPrintNowSpeed();
        break;
      case 2:
        // 운동 거리  디스플레이
        //        tftPrintTest2();
        tftPrintDistance();
        break;
      case 3:
        // 심박 수 디스플레이
        //        tftPrintTest3();
        tftPrintHeartRate();
        break;
      case 4:
        // 운동시간 디스플레이
        //        tftPrintTest3();
        tftPrintWorkoutTime();
        break;

    }
    lcdCnt++;
    tftTimeIndex = millis();
  }
}

void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}

/**
   심박수 디스플레이
*/
void tftPrintHeartRate() { //심박수 디스플레이 함수

  String message = "";
  if (!heartRateMeasureFlag && !heartRateLocationFlag) { // 부착중이지 않고 올바른 위치에 있지 않을때
    message = "E400"; // 에러코드 400
  }
  else if (heartRateMeasureFlag && !heartRateLocationFlag) { // 부착 중이지만 올바른 값이 아닐때
    message = "E401"; //에러코드 401
  }
  else if (heartRateMeasureFlag && heartRateLocationFlag) { //모든 조건이 만족됬을때
    message = String(globalHeartRate);
  }

  tft.setTextWrap(true);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(5, 20);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.println("Heart Rate");
  tft.setCursor(5, 40);
  tft.setTextSize(5);
  tft.println(message);
  tft.setCursor(5, 85);
  tft.setTextSize(2);
  tft.println("bpm");
}

void tftPrintCheckUser() {

  String message = "";
  //만약 사용자 태그가 인식되어 있다면
  if (userRFIDCheckFlag) {
    for (int i = 0; i < 4; i++) {
      message +=  nuidPICC[i];
    }
  } else { // 인식되어 있지 않다면
    message = "Please Contact Your ID Tag";
  }
  tft.setTextWrap(true);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(5, 20);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.println("Tag Info");
  tft.setCursor(5, 40);
  tft.setTextSize(2);
  tft.println(message);
}

void tftPrintDistance() {
  tft.setTextWrap(true);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(5, 20);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.println("Distance");
  tft.setCursor(5, 40);
  tft.setTextSize(5);
  tft.println(distanceUnitKm);
  tft.setCursor(5, 85);
  tft.setTextSize(2);
  tft.println("km");
}

void tftPrintNowSpeed() {
  tft.setTextWrap(true);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(5, 20);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.println("Speed");
  tft.setCursor(5, 40);
  tft.setTextSize(5);
  tft.println(roundSpeed);
  tft.setCursor(5, 85);
  tft.setTextSize(2);
  tft.println("km/h");
}

/**
   운동시간 디스플레이 함수
*/
void tftPrintWorkoutTime() {
  int tmp_time = (int)(workoutTime / 1000.0f); // 운동 시간 연산하기
  String message = String(tmp_time);
  tft.setTextWrap(true);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(5, 20);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(2);
  tft.println("WorkoutTime");
  tft.setCursor(5, 50);
  tft.setTextSize(5);
  tft.println(message);
  tft.setCursor(5, 100);
  tft.setTextSize(2);
  tft.println(" [s]");
}


//블루투스가 연결되고 실시간 전송 시 심박 수 처리 함수 - 박제창
void updateHeartRateLevel() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the charge level of a battery.
  */
  unsigned char hr = readHeartRate(HR_ADDR);
  heartRateData[1] = hr;
  heartRateMeansurement.setValue(heartRateData, 2);

#ifdef DEBUG
  Serial.print("HR -> "); Serial.println(hr, DEC);
  //delay(1000);
  //indoorBikeData[2] = (uint8_t)((uint16_t)(speedNow * 100) >> 8 & 0xFF);
  //indoorBikeData[3] = (uint8_t)((uint16_t)(speedNow * 100) & 0xFF);
#endif
}


/**
   심박센서로 심박수를 읽어오는 함수
   심박 센서와 i2c 통신이 필요하다.
*/
unsigned char readHeartRate(byte addr) {
  unsigned char c = 0;
  Wire.requestFrom(addr >> 1, 1);
  while (Wire.available()) {
    c = Wire.read();
  }
  return c;
}

