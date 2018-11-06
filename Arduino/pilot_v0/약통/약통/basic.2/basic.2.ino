#include "HX711.h"
HX711 scale(A2, A3); // DOUT, SCK

void setup()
{
  Serial.begin(9600);
  scale.set_scale(-1901.5); //// this value is obtained by calibrating the scale with known weights
  scale.tare();
}

void loop()
{
 float a = scale.get_units(); // 아날로그 0번핀에 연결된 압력센서의 값을 측정합니다.
  boolean b = 0;
  float sum = 0; // 합
  float ave = 0; // 평균
  if(!b) {
    for(int i =0; i<40; i++) 
    {
      sum = sum + a; // SUM에 압력센서값 더하기
      delay(80); // 0.1초 지연
    }
  ave = sum/40;  // 평균값 구하기
  }
  Serial.print("Average : "); // 평균값 모니터에 표시
  Serial.println(ave,1); 
  
 // Serial.println(a);
  delay(100);        // 0.1초마다 압력센서 값 출력
}

