/*
  On Board Indicator 
  GPIO 13

  @Author : Dreamwalker
*/

// 초기 한번 설정

void setup() {
  pinMode(13, OUTPUT);   // 13번 핀을 출력으로 설정 (설명문) 
}

// 반복 실행
void loop() {
  digitalWrite(13, HIGH);   // 디지털출력으로 13번 핀을 High로 출력
  delay(1000);    // 1000ms 동안 상태 유지 
  
  digitalWrite(13, LOW);    // 디지털출력으로 13번 핀을 Low로 출력 
  delay(1000);              // 1000ms 동안 상태 유지 
  }

