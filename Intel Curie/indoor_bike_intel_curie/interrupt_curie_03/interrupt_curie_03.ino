/**

   CODE : JAICHANGPARK
   인텔 큐리 보드 사용
   32비트 쿼크 코어
   에르고미터

*/

#include <CurieTime.h>
#include <Wire.h>
#include <CurieBLE.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "Crypto.h"

#define MAX_AES_PROCESS 32

#define HR_ADDR  0xA0
#define ONE_ROUND_DISTANCE 2.198F
#define LEXPA_DISTANCE 5.0F

#define REAL_TIME_STOP_MILLIS 3000
#define WORKOUT_DONE_TIME_MILLIS 30000

#define ERGOMETER_LEXPA
#define DEBUG
#define USE_OLED

#ifdef USE_OLED
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
#endif

BLEService heartRateService("180D"); // BLE Battery Service
BLECharacteristic heartRateMeansurement("2A37", BLERead | BLENotify , 2);     // remote clients will be able to
// get notifications if this characteristic changes
BLEService fitnessMachineService("1826"); // BLE FitnessMachine Service
BLECharacteristic indorBikeChar("2AD2", BLENotify, 4);
BLECharacteristic treadmillChar("2ACD", BLENotify, 5);

BLEService dateTimeService("AAA0");
BLECharacteristic dateTimeSyncChar("0001", BLERead | BLEWrite , 20);
BLECharacteristic dateChar("AAA1", BLERead | BLEWrite , 20);
BLECharacteristic timeChar("AAA2", BLERead | BLEWrite , 20);

BLEService userAuthService("EEE0");
BLECharacteristic authChar("EEE1", BLERead | BLEWrite , 20);

BLEService dataSyncService("FFF0");
BLECharacteristic controlChar("FFF1", BLERead | BLEWrite , 20);
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
    distanceUnitKm = distance / 1000.0f;
    InstantTime = (float)(intterrupt_time - t) / 1000.0f;
    speedNow = 3.6f * (LEXPA_DISTANCE / InstantTime);  // 현재 속도
    uintSpeedNow = (speedNow * 100);  // 애플리케이션으로 실시간 전송을 위해 100을 곱한다.
    uintTotalDistance = (uint32_t) distance; // 애플리케이션으로 실시간 운동 거리 전송을 위해 케스팅한다.
    workoutTime = intterrupt_time - startFitnessTime;
#else
    distance = (ONE_ROUND_DISTANCE * count); // 총 이동 거리
    distanceUnitKm = distance / 1000.0f;
    InstantTime = (float)(intterrupt_time - t) / 1000.0f;
    speedNow = 3.6f * (ONE_ROUND_DISTANCE / InstantTime);
    uintSpeedNow = (speedNow * 100);
    uintTotalDistance = distance * 100;
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
  dateTimeService.addCharacteristic(dateChar);
  dateTimeService.addCharacteristic(timeChar);

  BLE.setAdvertisedService(userAuthService);
  userAuthService.addCharacteristic(authChar);

  BLE.setAdvertisedService(dataSyncService);
  dataSyncService.addCharacteristic(controlChar);
  dataSyncService.addCharacteristic(syncChar);

  BLE.addService(heartRateService);   // Add the BLE Heart Rate service
  BLE.addService(fitnessMachineService);   // Add the BLE Indore Bike service
  BLE.addService(dateTimeService);
  BLE.addService(userAuthService);
  BLE.addService(dataSyncService);

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
  String message = String(distanceUnitKm) + " km";
  display.setTextSize(2);
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
  display.setTextSize(2);
  display.setTextColor(WHITE);
  String message = String(speedNow) + " km/h";
  display.println(message);
  display.display();
}

void displayPhaseThird() {
  display.clearDisplay();
  // text display tests
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println("Fitness Time");
  display.setTextSize(2);
  display.setTextColor(WHITE);
  int tmp_time = (int)(workoutTime / 1000.0f);
  String message = String(tmp_time) + " [s]";
  display.println(message);
  display.display();
}

#endif

void setup() {
  // put your setup code here, to run once:
#ifdef DEBUG
  Serial.begin(115200);
#endif
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(2, interrupt_func, FALLING);

  AES.Initialize(aes_iv, aes_key);

#ifdef USE_OLED
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  // init done
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  init_display();
#endif

  BLE.begin();
  BLE.setLocalName("KNU EG0");

  bleProfileSetUp();
  heartRateMeansurement.setValue(heartRateData, 2 );   // initial value for this characteristic
  indorBikeChar.setValue(indoorBikeData, 4);
  treadmillChar.setValue(treadmillData, 5);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  dateTimeSyncChar.setEventHandler(BLEWritten, datatimeSyncCharacteristicWritten);
  authChar.setEventHandler(BLEWritten, authCharCharacteristicWritten);

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

        updateBatteryLevel();

        uint8_t testPacket[] = {0x07, 0xe2, 0x09, 0x02, 0x0d, 0x2d, 0x00};
        syncChar.setValue(testPacket, 5);
      } else {
        //불루투스 연결은 되어 있지만 운동 종료 상황에서의 처리
      }
    }
  }

  // 블루투스 연결되어 있지 않은 상태에서 운동 중일 때 처리
  if (fitnessStartOrEndFlag) {
#ifdef USE_OLED
    int displayPhaseCounter = count % 3 ;
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
      }
#endif
    }

    // 운동 종료 시
    // 모든 변수 초기화 및 플레그 초기화
    // 저장 작업 처리
    // 운동 중 플레그가 high이고 (운동 중 이지만) 30초 동안 동작이 없으면 운동 종료 판단
    if (currentTimeIndicatorMillis - t > WORKOUT_DONE_TIME_MILLIS) {
      fitnessStartOrEndFlag = false;
      t = 0;
      count = 0;
      distance = 0;
      distanceUnitKm = 0;

      startFitnessTime = 0;
      endFitnessTime = 0;

#ifdef DEBUG
      Serial.println("운동 종료처리");
#endif
#ifdef USE_OLED
      init_display();
#endif
    }
  } else {
    // 블루투스 연결되어 있지 않은 상태이면서 운동 중이지 않을 때
    getNowTime();
    delay(1000);
  }

}


void updateBatteryLevel() {
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

unsigned char readHeartRate(byte addr) {
  unsigned char c = 0;
  Wire.requestFrom(addr >> 1, 1);
  while (Wire.available()) {
    c = Wire.read();
  }
  return c;
}


void blePeripheralConnectHandler(BLEDevice central) {
  digitalWrite(13, HIGH); // turn on the LED to indicate the connection:
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
  digitalWrite(13, LOW);
  deviceConnectedFlag = false;

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
  if (len != 0 && len <= 7 ) {
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
  AES.Decrypt(receiveData, 16, aes_result, 0);
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
    authData[1] = 0x00;
    authData[2] = 0x03;
    authChar.setValue(authData, 3);
  } else {
    authData[0] = 0x02;
    authData[1] = 0xFF;
    authData[2] = 0x03;
    authChar.setValue(authData, 3);
  }

  //  authCoreValue
}

void addReceiveBytes(const uint8_t* bytes, size_t len) {
  // note increment rxHead befor writing
  // so need to increment rxTail befor reading
  for (size_t i = 0; i < len; i++) {
    rxHead = (rxHead + 1) % sizeof(rxBuffer);
    rxBuffer[rxHead] = bytes[i];
  }
}

//void getNowTime() {
//  Serial.print("Time now is: ");
//  print2digits(hour());
//  Serial.print(":");
//  print2digits(minute());
//  Serial.print(":");
//  print2digits(second());
//
//  Serial.print(" ");
//
//  Serial.print(day());
//  Serial.print("/");
//  Serial.print(month());
//  Serial.print("/");
//  Serial.print(year());
//
//  Serial.println();
//}
//
//void print2digits(int number) {
//  if (number >= 0 && number < 10) {
//    Serial.print('0');
//  }
//  Serial.print(number);
//}

