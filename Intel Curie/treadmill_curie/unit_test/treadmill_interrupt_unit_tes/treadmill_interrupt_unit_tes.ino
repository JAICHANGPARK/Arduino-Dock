#include <CurieTime.h>
#include <CurieBLE.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Wire.h>
#include <SPI.h>

#define TREADMILL
#define INDOOR_BIKE
#define DEBUG


#define MAGNET_INTERRUPT_PIN          2
#define BLE_LED_INDICATOR_PIN         3

#define TREADMILL_DISTANCE            0.13F //단위 m 

volatile uint32_t count = 0;      // 인터럽트 클럭 카운트 수
volatile float distance = 0.0f;   // 거리 변수 [ m]
volatile float distanceUnitKm = 0.0f; // 거리 변수 [km]
volatile float speedNow = 0.0f; // 순간 속도 변수 [km/h]
volatile uint16_t uintSpeedNow = 0;
volatile uint32_t uintTotalDistance = 0;

volatile long t = 0;  // 센서 입력 외부 인터럽트 시간 변수 .
volatile float InstantTime = 0.0f;
volatile int diffTime = 0.0f;

long saveMinTime = 0;

/*************************************************************
   인터럽트 서비스 루틴 (ISR) 함수
   카운트 처리
   속도 연산
   거리 연산
   제작 : 박제창
 ***********************************************************/
void interrupt_func() {
#ifdef TREADMILL
  long interrupt_time = millis(); // 인터럽트가 발생한 시간을 저장
  count++; //자계 센서로 부터 외부인터럽트가 발생하면 1씩 증가시키도록
  distance = (TREADMILL_DISTANCE * count); // 총 이동 거리 --> 단위 넣을것
  distanceUnitKm = distance / 1000.0f; // 단위 km

  diffTime = interrupt_time - t; // millisecond의 시간차를 확인한다.

  InstantTime = (float)(interrupt_time - t) / 1000.0f;  // 과거의 시간과 현재의 시간을 뺀다.
  speedNow = 3.6f * (TREADMILL_DISTANCE / InstantTime);  // 현재 속도는 =  1회전 거리 / 1회전 발생 시간
  t = millis(); //시간 저장
#elif INDOORBIKE
#else
#endif
}

void setup() {

#ifdef DEBUG
  Serial.begin(115200);
#endif
  Wire.begin();
  attachInterrupt(MAGNET_INTERRUPT_PIN, interrupt_func, FALLING);
  pinMode(BLE_LED_INDICATOR_PIN, OUTPUT);
  digitalWrite(BLE_LED_INDICATOR_PIN, HIGH);
  delay(1000);
  digitalWrite(BLE_LED_INDICATOR_PIN, LOW);

}

void loop() {

  long currentTime = millis();
  if (currentTime - t >= 500) {
#ifdef DEBUG
    Serial.println("운동종료");
    count = 0;
#endif
  } else {
#ifdef DEBUG
    Serial.print("count -> "); Serial.print(count);
    Serial.print("| distance -> "); Serial.print(distance);
    Serial.print("| distanceUnitKm -> "); Serial.print(distanceUnitKm);
    Serial.print("| diffTime -> "); Serial.print(diffTime);
    Serial.print("| InstantTime -> "); Serial.print(InstantTime);
    Serial.print("| speedNow -> "); Serial.println(speedNow);
#endif
  }

  if (currentTime - saveMinTime >= 60000) {
    //    detachInterrupt(MAGNET_INTERRUPT_PIN);
    //    saveMinTime = 0;
  } else {

  }

}
