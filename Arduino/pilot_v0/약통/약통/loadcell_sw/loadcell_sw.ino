#include "HX711.h"             //로드셀 헤더
HX711 scale(A2, A3); // DOUT, SCK]      //로드셀 

int pushButton = 2;        //버튼

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
   if(66.6<=ave & ave<69){       //약먹기전
    Serial.println("sun");
    Serial.println("no");
    Serial.println(ave,1);
    delay(100);
    }
   else if(69<=ave & ave<70) {               //약통을 먹기위해 들었을때 & 약통open
         Serial.println("sat");
         Serial.println(ave,1);
         delay(100);
        }  
   else if(70<=ave & ave<71) {               //약통을 먹기위해 들었을때 & 약통open
        Serial.println("fri");
        Serial.println(ave,1);
        delay(100);
        }  
   else if(71<=ave & ave<72) {               //약통을 먹기위해 들었을때 & 약통open
         Serial.println("thr");
         Serial.println(ave,1);
         delay(100);
         }             
   else if(72<=ave & ave<73) {               //약통을 먹기위해 들었을때 & 약통open
        Serial.println("wen");
        Serial.println(ave,1);
        delay(100);
        }
   else if(73<=ave & ave<74) {               //약통을 먹기위해 들었을때 & 약통open
         Serial.println("tue");
         Serial.println(ave,1);
         delay(100);
        }    
   else if(74<=ave & ave<75.4) {               //약통을 먹기위해 들었을때 & 약통open
         Serial.println("mon");
         Serial.println(ave,1);
        delay(100);
        }  
   else if(75.4<=ave){               //약통을 먹기위해 들었을때 & 약통open
         Serial.println("full");
         Serial.println(ave,1);
         delay(100);
        }  
}  
 

