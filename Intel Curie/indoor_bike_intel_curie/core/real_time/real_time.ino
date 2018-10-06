/**

   CODE : JAICHANGPARK aka DREAMWALKER
   인텔 큐리 보드 사용
   32비트 쿼크 코어
   에르고미터

*/

#include <CurieTime.h>
#include <Wire.h>
#include <CurieBLE.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "CurieTimerOne.h"



#define MAX_AES_PROCESS 32

#define HR_ADDR  0xA0
#define ONE_ROUND_DISTANCE 2.198F
#define LEXPA_DISTANCE 5.0F

#define REAL_TIME_STOP_MILLIS 3000      // 실시간 3초 반응 없을 시 값 초기화
#define WORKOUT_DONE_TIME_MILLIS 30000   // 운동 자동 종료 시간 30초후 모든 정보 초기화 및 변수 저장.

#define DEBUG
#define ERGOMETER_LEXPA
#define USE_OLED

#ifdef USE_OLED
#define OLED_RESET 5
Adafruit_SSD1306 display(OLED_RESET);
#endif


#define BLE_LED_INDICATOR_PIN        4


BLEService heartRateService("180D"); // BLE Battery Service
BLECharacteristic heartRateMeansurement("2A37", BLERead | BLENotify , 2);     // remote clients will be able to
// get notifications if this characteristic changes
BLEService fitnessMachineService("1826"); // BLE FitnessMachine Service
BLECharacteristic indorBikeChar("2AD2", BLENotify, 4);
BLECharacteristic treadmillChar("2ACD", BLENotify, 5);

BLEService dateTimeService("AAA0");
BLECharacteristic dateTimeSyncChar("0001", BLERead | BLEWrite , 20);
//BLECharacteristic dateChar("AAA1", BLERead | BLEWrite , 20);
//BLECharacteristic timeChar("AAA2", BLERead | BLEWrite , 20);
BLECharacteristic datetimeChar("0002", BLERead | BLEWrite , 20);
BLECharacteristic resultChar("fff0", BLERead | BLEWrite | BLENotify, 4);

//BLEService resultService("BBB0");
//BLECharacteristic resultChar("0001", BLERead | BLEWrite , 5);

BLEService userAuthService("EEE0");
BLECharacteristic authChar("EEE1", BLERead | BLEWrite , 16);

BLEService dataSyncService("FFF0");
BLECharacteristic controlChar("FFF1", BLERead | BLEWrite , 10);
BLECharacteristic syncChar("FFF2", BLERead | BLEWrite | BLENotify , 20);

volatile uint32_t count = 0;
volatile float distance = 0.0f;
volatile float distanceUnitKm = 0.0f;
volatile float speedNow = 0.0f;
volatile uint16_t uintSpeedNow = 0;
volatile uint32_t uintTotalDistance = 0;

long t = 0;  // 센서 입력 외부 인터럽트 시간 변수 .
volatile float InstantTime = 0.0f;

long previousMillis = 0;  // last time the battery level was checked, in ms
long realTimePreviousMillis = 0;  // 실시간운동 정보 초기화를 위한 한계 시간 약 3초 이상 인터럽트가 없으면 초기화한다.

uint8_t heartRateData[] = {0b00000010, 0x00};
uint8_t treadmillData[] = {0x05, 0x00, 0x00, 0x00, 0x00};
uint8_t indoorBikeData[] = {0x00, 0x00, 0x00, 0x00};
uint8_t authData[] = {0x00, 0x00, 0x00};
uint8_t resultPacket[] = {0x00, 0x00, 0x00}; // 결과 값 반환 패킷 데이터 버퍼
boolean deviceConnectedFlag = false;  //장비가 블루투스 연결이 되었는지 확인하는 플래그
boolean fitnessStartOrEndFlag = false; // 운동의 시작과 종료를 판단하는 플레그


// 암호화
static byte aes_key[16] = {2, 2, 2, 2, 0, 0, 0, 0, 1, 1, 1, 1, 8, 8, 8, 8};
static byte aes_iv[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static byte aes_result[MAX_AES_PROCESS];
byte authCoreValue[] = "9876543210000001"; //16 chars == 16 bytes
byte encrypted[16]; // AHA! needs to be large, 2x is not enough

volatile long startFitnessTime = 0;
volatile long endFitnessTime = 0;
volatile long workoutTime = 0;

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

volatile int hr_cnt = 0;
volatile uint8_t hr_real_time = 0;
bool toggle = 0;                    // The LED status toggle
bool toggle1 = 0;

void timedBlinkIsr()   // callback function when interrupt is asserted
{
  digitalWrite(5, toggle);
  toggle = !toggle;  // use NOT operator to invert toggle value
  hr_cnt++;
}

void timedBlinkIsr2()   // callback function when interrupt is asserted
{
  digitalWrite(13, toggle);
  toggle1 = !toggle1;  // use NOT operator to invert toggle value
  hr_cnt = hr_cnt * 6;
  Serial.println( hr_cnt);
  hr_real_time = hr_cnt;
  hr_cnt = 0;
}


void interrupt_func() {
  if (count == 0) {
    fitnessStartOrEndFlag = true;
    startFitnessTime = millis();
  }
  long intterrupt_time = millis();

  // ISR에 들어온 시간 - 이전 저장 값
  // 클럭 카운트의 계수가 2개식 증가하는 현상을 잡아준다.
  if ( intterrupt_time - t >  50 ) {
    count++; //자계 센서로 부터 외부인터럽트가 발생하면 1씩 증가시키도록
#ifdef ERGOMETER_LEXPA
    distance = (LEXPA_DISTANCE * count); // 총 이동 거리 --> 단위 넣을것
    distanceUnitKm = distance / 1000.0f; // 단위 km
    uintDistanceKm = (distanceUnitKm * 100);

    InstantTime = (float)(intterrupt_time - t) / 1000.0f;
    speedNow = 3.6f * (LEXPA_DISTANCE / InstantTime);  // 현재 속도
    uintSpeedNow = (speedNow * 100);  // 애플리케이션으로 실시간 전송을 위해 100을 곱한다.
    uintTotalDistance = (uint32_t) distance; // 애플리케이션으로 실시간 운동 거리 전송을 위해 케스팅한다. 단위 m
    workoutTime = intterrupt_time - startFitnessTime;

    //평균 구하기위한 더하기 연산
    sumSpeed += uintSpeedNow;   //속도 합
    sumDistanceKm += uintDistanceKm; // 거리합 km를 100 곱한 값

#else
    distance = (ONE_ROUND_DISTANCE * count); // 총 이동 거리
    distanceUnitKm = distance / 1000.0f;
    InstantTime = (float)(intterrupt_time - t) / 1000.0f;
    speedNow = 3.6f * (ONE_ROUND_DISTANCE / InstantTime);
    uintSpeedNow = (speedNow * 100);
    uintTotalDistance = distance * 100;
    workoutTime = intterrupt_time - startFitnessTime;
#endif

    //  speedNow = 3.6f * (distance / (float)(intterrupt_time - t));

    t = millis(); //시간 저장
  }
}

void bleProfileSetUp() {
  BLE.setAdvertisedService(heartRateService);
  heartRateService.addCharacteristic(heartRateMeansurement);

  BLE.setAdvertisedService(fitnessMachineService);
  fitnessMachineService.addCharacteristic(indorBikeChar);
  fitnessMachineService.addCharacteristic(treadmillChar);

  BLE.setAdvertisedService(dateTimeService);
  dateTimeService.addCharacteristic(dateTimeSyncChar);
  dateTimeService.addCharacteristic(resultChar);
  //  dateTimeService.addCharacteristic(datetimeChar);

  //  BLE.setAdvertisedService(resultService);
  //  resultService.addCharacteristic(resultChar);

  BLE.setAdvertisedService(userAuthService);
  userAuthService.addCharacteristic(authChar);

  BLE.setAdvertisedService(dataSyncService);
  dataSyncService.addCharacteristic(controlChar);
  dataSyncService.addCharacteristic(syncChar);

  BLE.addService(heartRateService);   // Add the BLE Heart Rate service
  BLE.addService(fitnessMachineService);   // Add the BLE Indore Bike service
  BLE.addService(dateTimeService);  // 시간 동기화를 위한 서비스 추가
  BLE.addService(userAuthService);  // 사용자 인증을 위한 서비스 추가
  BLE.addService(dataSyncService);  // 데이터 동기화를 위한 서비스 추가
  //BLE.addService(resultService);  // 데이터 동기화를 위한 서비스 추가
}

#ifdef USE_OLED
void init_display() {
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);
  // Clear the buffer.
  display.clearDisplay();
  // text display tests
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Hello");
  display.println("Kick Pedal");
  display.display();
}

void displayPhaseFirst() {
  display.clearDisplay();
  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Distance");
  String message = String(distanceUnitKm) + "km";
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.println(message);
  display.display();
}

void displayPhaseSecond() {
  display.clearDisplay();
  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Speed");
//  display.setTextSize(1);
//  display.setTextColor(WHITE);
  String message = String(speedNow) + "km/h";
  display.println(message);
  display.display();
}

void displayPhaseThird() {
  display.clearDisplay();
  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("FitnessTime");
  display.setTextSize(1);
  display.setTextColor(WHITE);
  int tmp_time = (int)(workoutTime / 1000.0f); // 운동 시간 연산하기
  String message = String(tmp_time) + "[s]";
  display.println(message);
  display.display();
}

void displayPhaseFour() {
  display.clearDisplay();
  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("HeartRate");
  display.setTextSize(1);
  display.setTextColor(WHITE);

  //  boolean heartRateMeasureFlag = false;
  //  boolean heartRateLocationFlag = false;
  String message = "";
  if (!heartRateMeasureFlag && !heartRateLocationFlag) {
    message = "E400";
  }
  else if (heartRateMeasureFlag && !heartRateLocationFlag) {
    message = "E401";
  } else if (heartRateMeasureFlag && heartRateLocationFlag) {
    message = String(globalHeartRate) + " bpm";
  }
  display.println(message);
  display.display();

}


#endif

/**
   프로그램 초기화 1회 실행 - 박제창
*/
void setup() {
  pinMode(BLE_LED_INDICATOR_PIN, OUTPUT);
  digitalWrite(BLE_LED_INDICATOR_PIN, HIGH);
  delay(1000);
  digitalWrite(BLE_LED_INDICATOR_PIN, LOW);

  // put your setup code here, to run once:
#ifdef DEBUG
  Serial.begin(115200);
#endif
  Wire.begin();
  //  pinMode(LED_BUILTIN, OUTPUT);  -- 13번 핀 사용중이기 때문에 수정
  attachInterrupt(2, interrupt_func, FALLING);
  attachInterrupt(6, timedBlinkIsr, RISING);
  CurieTimerOne.start(10000000, &timedBlinkIsr2);  // set timer and callback

#ifdef USE_OLED
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  // init done
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  init_display();
#endif


  //저전력 블루투스 설정
  BLE.begin();
  BLE.setLocalName("KNU EG0");

  bleProfileSetUp();
  heartRateMeansurement.setValue(heartRateData, 2 );   // initial value for this characteristic
  indorBikeChar.setValue(indoorBikeData, 4);
  treadmillChar.setValue(treadmillData, 5);
  uint8_t initResultPacket[3] = {0,};
  initResultPacket[0] = 0x00;
  initResultPacket[1] = 0x00;
  initResultPacket[2] = 0x00;
  resultChar.setValue(initResultPacket, 3);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  dateTimeSyncChar.setEventHandler(BLEWritten, datatimeSyncCharacteristicWritten);
  authChar.setEventHandler(BLEWritten, authCharCharacteristicWritten);
  controlChar.setEventHandler(BLEWritten, dataSyncCharCharacteristicWritten);
  BLE.advertise(); // start advertising

}

void loop() {
  BLE.poll();

  long currentTimeIndicatorMillis = millis();

  // check the battery level every 200ms
  // as long as the central is still connected:
  if (deviceConnectedFlag) {

    // 불루투스 연결은 되어 있는 상태에서 운동 중일 때만 실시간 전송이 되도록
    if (fitnessStartOrEndFlag) {
      long currentMillis = millis();
      // if 200ms have passed, check the battery level:
      if (currentMillis - previousMillis >= 1000) {
        previousMillis = currentMillis;

        indoorBikeData[2] = (uint8_t)(uintSpeedNow & 0xFF);
        indoorBikeData[3] = (uint8_t)(uintSpeedNow >> 8) & 0xFF;
        indorBikeChar.setValue(indoorBikeData, 4);

        treadmillData[2] = (uint8_t)(uintTotalDistance & 0xFF);
        treadmillData[3] = (uint8_t)(uintTotalDistance >> 8) & 0xFF;
        treadmillData[4] = (uint8_t)(uintTotalDistance >> 16) & 0xFF;
        treadmillChar.setValue(treadmillData, 5);

        //        updateHeartRateLevel();

        updateHeartRate();

      } else {
        //불루투스 연결은 되어 있지만 운동 종료 상황에서의 처리
      }
    }

#ifdef DEBUG
    //todo : 연결 되면 보내지는 테스트 패킷
    uint8_t p1 = 0;
    uint8_t testPacket[] = {0x07, 0xe2, 0x09, 0x02, 0x0d, 0x2d, p1};
    //    syncChar.setValue(testPacket, 7);
    p1++;
#endif
  }

  // 블루투스 연결되어 있지 않은 상태에서 운동 중일 때 처리
  if (fitnessStartOrEndFlag) {
#ifdef USE_OLED
    int displayPhaseCounter = count % 4 ;
#endif

    long realTimeCurrentTimeMillis = millis();  // 현재 시스템 시간을 가져온다.
    // 운동 중이면서 만약 3초동안 인터럽트 발생이 없다면 실시간 운동 변수 초기화
    if (realTimeCurrentTimeMillis - t > REAL_TIME_STOP_MILLIS) {
      InstantTime = 0;
      speedNow = 0;
      uintSpeedNow = 0;
#ifdef DEBUG
      //    Serial.println("실시간 운동 종료 ");
#endif
    } else {
      // 운동 중이면서 3초 이내로 인터럽트가 발생한다면 실시간 운동 변수 표기

      //checkHeartRateInWorkout();
      unsigned char hr_realtime = readHeartRate(HR_ADDR);
      //    uint16_t heartRateCount = 0;
      //boolean heartRateMeasureFlag = false;
      //boolean heartRateLocationFlag = false;
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

      delay(100);

#ifdef DEBUG
      Serial.print("count -> "); Serial.print(count); Serial.print("| instant Time  -> "); Serial.print(InstantTime);
      Serial.print("| workout Time  -> "); Serial.print(workoutTime);
      Serial.print("| distance -> "); Serial.print(distance);   Serial.print("| distance m to Km-> "); Serial.print(distanceUnitKm);
      Serial.print(" | Speed ->"); Serial.print(speedNow); Serial.print(" | Speed * 100 ->"); Serial.println(uintSpeedNow);
#endif

#ifdef USE_OLED
      switch (displayPhaseCounter) {
        case 0:
          displayPhaseFirst();
          break;
        case 1:
          displayPhaseSecond();
          break;
        case 2:
          displayPhaseThird();
          break;
        case 3:
          displayPhaseFour();
          break;
      }
#endif
    }

    // 운동 종료 시
    // 모든 변수 초기화 및 플레그 초기화
    // 저장 작업 처리
    // 운동 중 플레그가 high이고 (운동 중 이지만) 30초 동안 동작이 없으면 운동 종료 판단
    if (currentTimeIndicatorMillis - t > WORKOUT_DONE_TIME_MILLIS) {
      // 운동 종료시 운동량 데이터 저장 처리

      uint16_t saveYear = year();
      uint8_t saveMonth = month();
      uint8_t saveDay = day();
      uint8_t saveHour = hour();
      uint8_t saveMin = minute();
      uint8_t saveSecond = second();

      uint16_t meanSpeed = sumSpeed / count; //카운트 수를 기준으로 평균 운동 기록 연산
      int tmpTime = (int)(workoutTime / 1000.0f);
      uint16_t saveWorkoutTime = (uint16_t)tmpTime; //운동 시간 변수
      // 이동 거리는 --> uintDistanceKm 변수 사용


#ifdef DEBUG
      //      Serial.print("저장 주소 --> ");  Serial.print("0x"); Serial.println(saveAddress, HEX);
      Serial.print("인덱스 --> "); Serial.print((uint8_t)((register_index >> 8) & 0xff), HEX); Serial.println((uint8_t)(register_index & 0xff), HEX); //2바이트
      Serial.print("저장 시간 --> "); Serial.print((uint8_t)((saveYear >> 8) & 0xff), HEX); Serial.print((uint8_t)(saveYear & 0xff), HEX); //2바이트
      Serial.print(saveMonth, HEX); //1바이트
      Serial.print(saveDay, HEX);   //1바이트
      Serial.print(saveHour, HEX);  //1바이트
      Serial.print(saveMin, HEX);   //1바이트
      Serial.println(saveSecond, HEX); //1바이트
      Serial.print("운동시간 --> "); Serial.println(saveWorkoutTime); // 2바이트
      Serial.print("평균 속도 --> "); Serial.println(meanSpeed); // 2바이트
      Serial.print("총 운동 거리 --> "); Serial.println(uintDistanceKm);  // 2바이트
      //      Serial.print("평균 심박수 --> "); Serial.println(meanHeartRate);  //1바이트
      Serial.print("소모 열량 --> "); Serial.println(0); //2바이트

#endif
      register_index++;

      // 운동 종료시 모든 변수 초기화
      fitnessStartOrEndFlag = false;
      t = 0;
      count = 0;
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

#ifdef DEBUG
      Serial.println("운동 종료처리");
#endif
#ifdef USE_OLED
      init_display();
#endif
    }
  } else {
    // 블루투스 연결되어 있지 않은 상태이면서 운동 중이지 않을 때

#ifdef DEBUG

    //    device_id = getDeviceID();
    Serial.print("id: "); Serial.println(device_id, HEX);
    //    getNowTime();

    unsigned char hr_test = readHeartRate(HR_ADDR);
    //    uint16_t heartRateCount = 0;
    //boolean heartRateMeasureFlag = false;
    //boolean heartRateLocationFlag = false;
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


    if (bleDateTimeSycnFlag  && bleAuthCheckFlag  && bleDataSyncFlag ) {

      for (int i = 0; i < register_index; i++) {

      }
      resultPacket[0] = 0x02;
      resultPacket[1] = 0x03;
      resultPacket[2] = 0x03;
      resultChar.setValue(resultPacket, 3);
      bleDateTimeSycnFlag = false;
      bleAuthCheckFlag = false;
      bleDataSyncFlag = false;
    }
    //-----------------------------
    delay(1000);
#endif
  }
}

void checkHeartRateInWorkout() {
  unsigned char hr_test = readHeartRate(HR_ADDR);
  //    uint16_t heartRateCount = 0;
  //boolean heartRateMeasureFlag = false;
  //boolean heartRateLocationFlag = false;
  if (hr_test == 0) {
    heartRateMeasureFlag = false;
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

    // 심박수 카운트 업 , 심박수 평균을 위한 더하기 수행 --> 운동 종료시 초기화 될 변수
    heartRateCount++;
    sumHeartRate += globalHeartRate;
  }
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


//블루투스가 연결되고 실시간 전송 시 심박 수 처리 함수 - 박제창
void updateHeartRate() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the charge level of a battery.
  */
  unsigned char hr = 0x00;
  if (hr_real_time == 0) {
    hr = 0xFF;
  } else {
    hr = hr_real_time;
  }
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


/******************************************************************************
                                 Bluetooth Callback Function
                                제작 : 박제창
 ***************************/



void blePeripheralConnectHandler(BLEDevice central) {
  //  digitalWrite(13, HIGH); // turn on the LED to indicate the connection:
  digitalWrite(BLE_LED_INDICATOR_PIN, HIGH);
  deviceConnectedFlag = true;
#ifdef DEBUG
  Serial.print("Connected to central: ");
  Serial.println(central.address()); // print the central's MAC address:
#endif
}

/**
    // central disconnected event handler
*/
void blePeripheralDisconnectHandler(BLEDevice central) {
  // when the central disconnects, turn off the LED:
  //  digitalWrite(13, LOW);
  digitalWrite(BLE_LED_INDICATOR_PIN, LOW);
  deviceConnectedFlag = false;

  // 연결 종료후 패킷 초기화
  resultPacket[0] = 0x00;
  resultPacket[1] = 0x00;
  resultPacket[2] = 0x00;
  resultChar.setValue(resultPacket, 3);

#ifdef DEBUG
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
#endif
}

int rxHead = 0;
static uint8_t rxBuffer[20];

void datatimeSyncCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED
  size_t len = dateTimeSyncChar.valueLength();
#ifdef DEBUG
  Serial.println("Characteristic event, written: datatimeSyncCharacteristicWritten ");
  Serial.print("len --> "); Serial.println(len);
#endif
  const unsigned char *data = dateTimeSyncChar.value();
  //  addReceiveBytes(data, len);
#ifdef DEBUG
  for (size_t i = 0; i < len; i++) {
    Serial.println(data[i], HEX);
  }
#endif

  if (len != 0 && len == 7 ) {
    int receiveYear = ((data[0] << 8 ) & 0xff00 | (data[1] & 0xff));
    int receiveMonth = (data[2] & 0xff);
    int receiveDay = (data[3] & 0xff);
    int receiveHour = (data[4] & 0xff);
    int receiveMinute = (data[5] & 0xff);
    int receiveSecond = (data[6] & 0xff);
    setTime(receiveHour, receiveMinute, receiveSecond, receiveDay, receiveMonth, receiveYear); // 시, 분, 초 ,일, 월, 년
#ifdef DEBUG
    Serial.print("날짜 값 변환 --> "); Serial.println(receiveYear);
#endif
    resultPacket[0] = 0x02;
    resultPacket[1] = 0x01;
    resultPacket[2] = 0x03;
    resultChar.setValue(resultPacket, 3);
    bleDateTimeSycnFlag = true;
  } else {
    //날짜 정보가 올바르지 않은 경우
    resultPacket[0] = 0x02;
    resultPacket[1] = 0xff;
    resultPacket[2] = 0x03;
    resultChar.setValue(resultPacket, 3);
    bleDateTimeSycnFlag = false;
  }
}

void authCharCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED

  size_t len = authChar.valueLength();
#ifdef DEBUG
  Serial.println("Characteristic event, written: authCharCharacteristicWritten ");
  Serial.print("len --> "); Serial.println(len);
#endif
  const unsigned char *data = authChar.value();
  byte receiveData[16];
  byte aes_result[16];

  for (int i = 0; i < 16; i++) {
    receiveData[i] = data[i];
  }
#ifdef DEBUG
  for (size_t i = 0; i < len; i++) {
    Serial.print(data[i], HEX);
    Serial.print(",");
  }
#endif
  //  AES.Decrypt(receiveData, 16, aes_result, 0);
#ifdef DEBUG
  Serial.println("Decrypt");
  for (int i = 0; i < 16; i++) {
    Serial.print(aes_result[i], HEX);
    Serial.print(",");
  }
#endif
  boolean authFlag = false;
  for (int i = 0; i < 16; i++) {
    if (!authCoreValue[i] == aes_result[i]) {
      //      Serial.println("Error");
      authFlag = false;

    } else {
      //      Serial.print("SUCESS");
      authFlag = true;
    }
  }
  if (authFlag) {
    authData[0] = 0x02;
    authData[1] = 0x02;
    authData[2] = 0x03;
    resultChar.setValue(authData, 3);
    bleAuthCheckFlag = true;
  } else {
    authData[0] = 0x02;
    authData[1] = 0xFF;
    authData[2] = 0x03;
    resultChar.setValue(authData, 3);
    bleAuthCheckFlag = false;
  }

  //  authCoreValue
}


/**
    데이터 동기화를 위한 콜백함수 - 박제창
    데이터 전송 요청
    0x00 : 전부
    0xyy : 아직 정해지지 않음 Reserved
  boolean bleDateTimeSycnFlag = false // 블루투스를 통해 시간 동기화가 되었을시 처리하는 플래그
  boolean bleAuthCheckFlag = false // 사용자 인증을 위한 플래그 실패시 false 성공시 true
  boolean bleDataSyncFlag = false // 데이터 전송 요청 이 들어왔을겨우 올바른 데이터 형식이면 true 아니면 false

*/
void dataSyncCharCharacteristicWritten(BLEDevice central, BLECharacteristic characteristic) {
  // central wrote new value to characteristic, update LED

  size_t len = controlChar.valueLength();
#ifdef DEBUG
  Serial.println("Characteristic event, written: dataSyncCharCharacteristicWritten ");
  Serial.print("len --> "); Serial.println(len);
#endif
  const unsigned char *data = controlChar.value();

}


void addReceiveBytes(const uint8_t* bytes, size_t len) {
  // note increment rxHead befor writing
  // so need to increment rxTail befor reading
  for (size_t i = 0; i < len; i++) {
    rxHead = (rxHead + 1) % sizeof(rxBuffer);
    rxBuffer[rxHead] = bytes[i];
  }
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



