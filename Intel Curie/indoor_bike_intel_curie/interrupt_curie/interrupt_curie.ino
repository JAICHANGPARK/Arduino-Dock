/**

   CODE : JAICHANGPARK
   인텔 큐리 보드 사용
   32비트 쿼크 코어

*/

#include <Wire.h>
#include <CurieBLE.h>
#define HR_ADDR  0xA0
#define ONE_ROUND_DISTANCE 2.198F
#define LEXPA_DISTANCE 5.0F

#define ERGOMETER_LEXPA
#define DEBUG

BLEService heartRateService("180D"); // BLE Battery Service
BLECharacteristic heartRateMeansurement("2A37", BLERead | BLENotify , 2);     // remote clients will be able to
// get notifications if this characteristic changes
BLEService fitnessMachineService("1826"); // BLE FitnessMachine Service
BLECharacteristic indorBikeChar("2AD2", BLENotify, 4);
BLECharacteristic treadmillChar("2ACD", BLENotify, 5);


volatile uint32_t count = 0;
volatile float distance = 0.0f;
volatile float speedNow = 0.0f;
volatile uint16_t uintSpeedNow = 0;
volatile uint32_t uintTotalDistance = 0;

long t = 0;
volatile float InstantTime = 0.0f;

long previousMillis = 0;  // last time the battery level was checked, in ms

uint8_t heartRateData[] = {0b00000010, 0x00};
uint8_t treadmillData[] = {0x05, 0x00, 0x00, 0x00, 0x00};
uint8_t indoorBikeData[] = {0x00, 0x00, 0x00, 0x00};

void interrupt_func() {
  long intterrupt_time = millis();
  // ISR에 들어온 시간 - 이전 저장 값
  // 클럭 카운트의 계수가 2개식 증가하는 현상을 잡아준다.
  if ( intterrupt_time - t >  50 ) {
    count++;
#ifdef ERGOMETER_LEXPA
    distance = (LEXPA_DISTANCE * count); // 총 이동 거리
    InstantTime = (float)(intterrupt_time - t) / 1000.0f;
    speedNow = 3.6f * (LEXPA_DISTANCE / InstantTime);
    uintSpeedNow = (speedNow * 100);
    uintTotalDistance = (uint32_t) distance;
#else
    distance = (ONE_ROUND_DISTANCE * count); // 총 이동 거리
    InstantTime = (float)(intterrupt_time - t) / 1000.0f;
    speedNow = 3.6f * (ONE_ROUND_DISTANCE / InstantTime);
    uintSpeedNow = (speedNow * 100);
    uintTotalDistance = distance * 100;
#endif

    //  speedNow = 3.6f * (distance / (float)(intterrupt_time - t));

    t = millis(); //시간 저장
  }
}

void setup() {
  // put your setup code here, to run once:
#ifdef DEBUG
  Serial.begin(115200);
#endif
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  attachInterrupt(2, interrupt_func, FALLING);

  BLE.begin();
  BLE.setLocalName("KNU EG0");
  BLE.setAdvertisedService(heartRateService);  // add the service UUID
  heartRateService.addCharacteristic(heartRateMeansurement); // add the Heart Rate characteristic

  BLE.setAdvertisedService(fitnessMachineService);  // add the service UUID
  fitnessMachineService.addCharacteristic(indorBikeChar); // add the ergometer level characteristic
  fitnessMachineService.addCharacteristic(treadmillChar); // add the ergometer level characteristic

  BLE.addService(heartRateService);   // Add the BLE Heart Rate service
  BLE.addService(fitnessMachineService);   // Add the BLE Indore Bike service

  heartRateMeansurement.setValue(heartRateData, 2 );   // initial value for this characteristic
  indorBikeChar.setValue(indoorBikeData, 4);
  treadmillChar.setValue(treadmillData, 5);
  BLE.advertise(); // start advertising
}

void loop() {
  BLEDevice central = BLE.central();
  // if a central is connected to peripheral:
  if (central) {
#ifdef DEBUG
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
#endif
    // turn on the LED to indicate the connection:
    digitalWrite(13, HIGH);

    // check the battery level every 200ms
    // as long as the central is still connected:
    while (central.connected()) {
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
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(13, LOW);
#ifdef DEBUG
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
#endif
  }
#ifdef DEBUG
  Serial.print("count -> "); Serial.print(count); Serial.print("| instant Time  -> "); Serial.print(InstantTime);
  Serial.print("| distance -> "); Serial.print(distance); Serial.print(" | Speed ->"); Serial.print(speedNow);
  Serial.print(" | Speed * 100 ->"); Serial.println(uintSpeedNow);
#endif
}


void updateBatteryLevel() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the charge level of a battery.
  */
  //int battery = analogRead(A0);
  // int batteryLevel = map(battery, 0, 1023, 0, 100);

  //  if (batteryLevel != oldBatteryLevel) {      // if the battery level has changed
  //    Serial.print("Battery Level % is now: "); // print it
  //    Serial.println(batteryLevel);
  //    batteryLevelChar.setValue(batteryLevel);  // and update the battery level characteristic
  //    oldBatteryLevel = batteryLevel;           // save the level for next comparison
  //  }
  unsigned char hr = readHeartRate(HR_ADDR);
  heartRateData[1] = hr;
  heartRateMeansurement.setValue(heartRateData, 2);

  Serial.print("HR -> "); Serial.println(hr, DEC);
  //delay(1000);
  //indoorBikeData[2] = (uint8_t)((uint16_t)(speedNow * 100) >> 8 & 0xFF);
  //indoorBikeData[3] = (uint8_t)((uint16_t)(speedNow * 100) & 0xFF);

}

unsigned char readHeartRate(byte addr) {
  unsigned char c = 0;
  Wire.requestFrom(addr >> 1, 1);
  while (Wire.available()) {
    c = Wire.read();
  }
  return c;
}

