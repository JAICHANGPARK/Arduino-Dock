#include "HX711.h"
HX711 scale(A2, A3); // DOUT, SCK
#include <Wire.h>
#include <RtcDS3231.h>
RtcDS3231<TwoWire> Rtc(Wire);
#include <SD.h>

File myFile;

int Year;
// UNO 데이터핀 연결
// SCL - pin A5
// SDA - pin A4
//

void setup()
{
  Serial.begin(9600);
  scale.set_scale(-1901.5); //// this value is obtained by calibrating the scale with known weights
  scale.tare();

   Serial.println("-----------------weight-------------------");
   Rtc.Begin();

    RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  if (!Rtc.IsDateTimeValid())
  {

    Serial.println("RTC lost confidence in the DateTime!");

    Rtc.SetDateTime(compiled);
  }

  if (!Rtc.GetIsRunning())
  {
    Serial.println("RTC was not actively running, starting now");
    Rtc.SetIsRunning(true);
  }

  RtcDateTime now = Rtc.GetDateTime();

  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);

  pinMode(10, OUTPUT);     //sd

  //SD카드 확인
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
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
      delay(100); // 0.1초 지연
    }
  ave = sum/40;  // 평균값 구하기
  }

  //파일이름
  myFile = SD.open("test.txt", FILE_WRITE);
  myFile.print("date       ");
  myFile.println("time");
  
 if(67<=ave & ave<68.9){       //일요일
    Serial.println("sun");
    Serial.println("no");
    //Serial.println(ave,1);
    if (!Rtc.IsDateTimeValid())                            //주입량 시간 시리얼 모니터 출력
    {
      Serial.println("RTC lost confidence in the DateTime!");
    }

    RtcDateTime now = Rtc.GetDateTime();
    printDateTime_sc(now);
    Serial.println();

    myFile.print("sun");
    printDateTime_sd(now);
    myFile.println();
    
    delay(100);
    }
    
   else if(69<=ave & ave<70.9) {               //토요일
         Serial.println("sat");
         //Serial.println(ave,1);
         if (!Rtc.IsDateTimeValid())                            //주입량 시간 시리얼 모니터 출력
    {
      Serial.println("RTC lost confidence in the DateTime!");
    }

    RtcDateTime now = Rtc.GetDateTime();
    printDateTime_sc(now);
    Serial.println();

    myFile.print("sat");
    printDateTime_sd(now);
    myFile.println();
    
         delay(100);
        }  
        
   else if(71<=ave & ave<72.4) {               //금요일
        Serial.println("fri");
        //Serial.println(ave,1);
        if (!Rtc.IsDateTimeValid())                            //주입량 시간 시리얼 모니터 출력
    {
      Serial.println("RTC lost confidence in the DateTime!");
    }

    RtcDateTime now = Rtc.GetDateTime();
    printDateTime_sc(now);
    Serial.println();

    myFile.print("fri");
    printDateTime_sd(now);
    myFile.println();
    
        delay(100);
        }  
        
   else if(72.4<=ave & ave<73.5) {               //목요일
         Serial.println("thr");
        //Serial.println(ave,1);
         if (!Rtc.IsDateTimeValid())                            //주입량 시간 시리얼 모니터 출력
    {
      Serial.println("RTC lost confidence in the DateTime!");
    }

    RtcDateTime now = Rtc.GetDateTime();
    printDateTime_sc(now);
    Serial.println();

    myFile.print("thr");
    printDateTime_sd(now);
    myFile.println();
    
         delay(100);
         }             
         
   else if(73.5<=ave & ave<75.3) {               //수요일
        Serial.println("wen");
        //Serial.println(ave,1);
        if (!Rtc.IsDateTimeValid())                            //주입량 시간 시리얼 모니터 출력
    {
      Serial.println("RTC lost confidence in the DateTime!");
    }

    RtcDateTime now = Rtc.GetDateTime();
    printDateTime_sc(now);
    Serial.println();

    myFile.print("wen");
    printDateTime_sd(now);
    myFile.println();
    
        delay(100);
        }
        
   else if(75.4<=ave & ave<76.6) {               //화요일
         Serial.println("tue");
         //Serial.println(ave,1);
         if (!Rtc.IsDateTimeValid())                            //주입량 시간 시리얼 모니터 출력
    {
      Serial.println("RTC lost confidence in the DateTime!");
    }

    RtcDateTime now = Rtc.GetDateTime();
    printDateTime_sc(now);
    Serial.println();

    myFile.print("tue");
    printDateTime_sd(now);
    myFile.println();
    
         delay(100);
        }    
        
   else if(76.7<=ave & ave<78.1) {               //월요일
         Serial.println("mon");
         //Serial.println(ave,1);
         if (!Rtc.IsDateTimeValid())                            //주입량 시간 시리얼 모니터 출력
    {
      Serial.println("RTC lost confidence in the DateTime!");
    }

    RtcDateTime now = Rtc.GetDateTime();
    printDateTime_sc(now);
    Serial.println();

    myFile.print("mon");
    printDateTime_sd(now);
    myFile.println();
    
        delay(100);
        }  
   else if(78.2<=ave){               //full
         Serial.println("full");
         //Serial.println(ave,1);
         if (!Rtc.IsDateTimeValid())                            //주입량 시간 시리얼 모니터 출력
    {
      Serial.println("RTC lost confidence in the DateTime!");
    }

    RtcDateTime now = Rtc.GetDateTime();
    printDateTime_sc(now);
    Serial.println();
         delay(100);
        }  
        myFile.close();
}

#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime_sc(const RtcDateTime& dt)
{
  char datestring[20];

  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u.%02u.%02u-%02u.%02u.%02u"),
             dt.Year(),
             dt.Month(),
             dt.Day(),
             dt.Hour(),
             dt.Minute(),
             dt.Second() );
  Serial.print(datestring);
}

void printDateTime_sd(const RtcDateTime& dt)
{
  char datestring[20];

  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u%02u%02u%02u%02u%02u"),
             dt.Year(),
             dt.Month(),
             dt.Day(),
             dt.Hour(),
             dt.Minute(),
             dt.Second() );
 myFile.print(datestring);
}

