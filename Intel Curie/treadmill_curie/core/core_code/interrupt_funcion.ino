/*************************************************************
   인터럽트 서비스 루틴 (ISR) 함수
   카운트 처리
   속도 연산
   거리 연산
   @author: 박제창
 ***********************************************************/
void interrupt_func() {
  if (count == 0) {
    fitnessStartOrEndFlag = true;
    startFitnessTime = millis();
  }

#ifdef TREADMILL
  long interrupt_time = millis(); // 인터럽트가 발생한 시간을 저장
  count++; //자계 센서로 부터 외부인터럽트가 발생하면 1씩 증가시키도록
  distance = (TREADMILL_DISTANCE * count); // 총 이동 거리 --> 단위 넣을것
  distanceUnitKm = distance / 1000.0f; // 단위 km

  diffTime = interrupt_time - t; // millisecond의 시간차를 확인한다.

  InstantTime = (float)(interrupt_time - t) / 1000.0f;  // 과거의 시간과 현재의 시간을 뺀다.
  speedNow = 3.6f * (TREADMILL_DISTANCE / InstantTime);  // 현재 속도는 =  1회전 거리 / 1회전 발생 시간
  uintSpeedNow = (speedNow * 100);  // 애플리케이션으로 실시간 전송을 위해 100을 곱한다.
  uintTotalDistance = (uint32_t) distance; // 애플리케이션으로 실시간 운동 거리 전송을 위해 케스팅한다. 단위 m

  workoutTime = interrupt_time - startFitnessTime;
  
  t = millis(); //시간 저장
#elif INDOORBIKE
#else
#endif
}
