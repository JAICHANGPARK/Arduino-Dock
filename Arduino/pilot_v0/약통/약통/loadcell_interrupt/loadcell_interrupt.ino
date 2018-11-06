#include "HX711.h"             //로드셀 헤더
HX711 scale(A2, A3); // DOUT, SCK]      //로드셀 

int pushButton = 2;        //버튼

boolean bswOn=false;
boolean bscaleOn=false;

void setup()  
{
  Serial.begin(9600);         //시리얼 모니터
  scale.set_scale(-523.5); //// 로드셀 영점 조정
  scale.tare();       //로드셀
  
   pinMode(pushButton, INPUT);      //버튼 INPUT
   Serial.println("-----------------weight-------------------");
}

void loop() {

 int sw = digitalRead(pushButton);     //버튼 

 float a = scale.get_units(); // 아날로그 0번핀에 연결된 압력센서의 값을 측정합니다.
  boolean b = 0;
  float sum = 0; // 합
  float ave; // 평균
     
  if(!b) {
    for(int i =0; i<40; i++) 
    {
      sum = sum + a; // SUM에 압력센서값 더하기
      delay(10); // 0.1초 지연
    }
   ave = sum/40;  // 평균값 구하기
  }
 if(sw==0){
  if(71<=ave & ave<72) {  
    Serial.println("sun");
    Serial.println(ave,1);
     delay(100);             //약통을 먹기위해 들었을때 & 약통open
        }     
 }
 delay(1000);
}
         
//void swlnterrrupt(){
//   if(digitalRead(pushButton) == HIGH){
//    if(bswOn==false){
//      bswOn=true;
//      bscaleOn=scale.get_units();
      
//    }
//   }
//}

