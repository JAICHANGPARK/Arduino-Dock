
/*  sd 카드 : micro sd  d0 12 / sdk 13  / d1 11 / cs 7
    RFID : SDA 10 / SCK 13 / MOSI 11 / MISO 12 / RST 9
    blu     :   rxd 5 /  txd 4
    interrupter :  3
     rtc   :         scl a5 / sda a4
     심박 :  2   긴게cl
    부저 : 8
*/

#include <Wire.h>
#include <RtcDS3231.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <MFRC522.h>


// RFID핀 정의
#define CS_RFID 10
#define RST_RFID 9
// SD카드 핀 정의
#define CS_SD 7


LiquidCrystal_I2C lcd(0x27, 20, 4);   // 접근주소: 0x3F or 0x27
MFRC522 rfid(CS_RFID, RST_RFID);


File myFile;

int blueTx = 4;
int blueRx = 5;
SoftwareSerial BTserial(blueTx, blueRx);

volatile int cnt, cnt_jump = 0;
volatile int cnt2 = 2;
volatile float t = 0.0f;
volatile float index = 1;

float av_spd, spd_add, spd, distance;
int a, b, c = 0;
char datasig;

float distance_add = 0.0f;

char sddata;

//심박관련
volatile int BPM;                   // 심박수 저장
volatile int Signal;                // 심장박동센서에서 측정되는 값 저장
volatile int IBI = 600;             // 심박수 측정 시 사용되는 시간 변수(심장이 몇초마다 뛰는지 측정)
volatile boolean Pulse = false;     // 유저의 심박수가 측정되면 True, 아무것도 측정되지 않으면 False
volatile int BPM_add = 0;
volatile int BPM_av = 0;


//칼로리관련
float kcal = 0;
int weight = 70; //   몸무개 설정
float mets = 3.1; //   5km 기준 3,1 , 5.5 → 3.1, 6.5 → 5

//RFID
String uidString;
int RFID_ONE = 0;
int id = 0;

RtcDS3231<TwoWire> Rtc(Wire);


//인터럽터 카운트(cnt 측정 및 거리, 현재속도 계산)
void count(void) {
  cnt++;
  distance = ((cnt/36 * 0.06)/1000);       //[km] 한바퀴 홈 36개 한바퀴당 6cm 이동  //  펙트
  spd = 3.6f * (float)((float)((distance*1000) / (float)2.0));   //[km/h]  현재속도  완료
}


//RFID 읽기
void readRFID() {

  rfid.PICC_ReadCardSerial();
  Serial.println(F("RFID 데이터 받는부분"));
  uidString = String(rfid.uid.uidByte[0]);
  digitalWrite(8, HIGH);
  
  delay(30);
  digitalWrite(8, LOW);
  if (uidString == "86") {
    id = 1;
    Serial.println("id 1");
  }
  if (uidString == "109"){
    id = 2;
    Serial.println("id 2");
  }
}
///////////////시간 SD 로깅 /////////////////////////////////////
void time_sd() {
  if (int(t) < 10) {
    myFile.print("000");
    myFile.print(int(t));
  }
  if (10 <= int(t) && int(t) < 100) {
    myFile.print("00");
    myFile.print(int(t));
  }
  if (100 <= int(t) && int(t) < 1000) {
    myFile.print("0");
    myFile.print(int(t));
  }
  if (1000 <= int(t) && int(t) <10000) {
    myFile.print(int(t));
  }
}
////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
void bpm() {
  if(BPM_av < 10){
    myFile.print("00");
    myFile.print(BPM_av);
  }
  if(10 <= BPM_av && BPM_av < 100){
    myFile.print("0");
    myFile.print(BPM_av);
  }
  if(100 <= BPM_av && BPM_av < 1000) {
    myFile.print(BPM_av);
  }
}
////////////USER에따라 파일 오픈(쓰기)/////////////////////
void sd_write(){
  if(id == 0){
    myFile = SD.open("data_0.txt", FILE_WRITE);
    Serial.println(F("data_0 open"));
  }
  if(id == 1){
    myFile = SD.open("data_1.txt",FILE_WRITE);   
  Serial.println(F("data_1 open"));
  }
  if(id == 2){
    myFile = SD.open("data_2.txt",FILE_WRITE);
    Serial.println(F("data_2 open"));
  }
}
//////////////////////////////////////////////////////
////////////USER에따라 파일 오픈(읽기)/////////////////////
void sd_read(){
  if(id == 0){
    myFile = SD.open("data_0.txt");
    Serial.println(F("data_0 open"));
  }
  if(id == 1){
    myFile = SD.open("data_1.txt");   
  Serial.println(F("data_1 open"));
  }
  if(id == 2){
    myFile = SD.open("data_2.txt");
    Serial.println(F("data_2 open"));
  }
}
//////////////////////////////////////////////////////


void setup () {

  Serial.begin(9600);
  BTserial.begin(9600);
  SPI.begin();

  Wire.begin();
  rfid.PCD_Init();


  //LCD
  lcd.init();
  lcd.backlight();


  //심박관련
  interruptSetup();


  //RTC 설정
  Rtc.Begin();
  RtcDateTime compiled = RtcDateTime(__DATE__, __TIME__);
  RtcDateTime now = Rtc.GetDateTime();

  Rtc.Enable32kHzPin(false);
  Rtc.SetSquareWavePin(DS3231SquareWavePin_ModeNone);


  //cnt 인터럽트
  attachInterrupt(digitalPinToInterrupt(3), count, FALLING);
  pinMode(3, INPUT);
  pinMode(8, OUTPUT);//부저


  //SD카드 확인
  if (!SD.begin(CS_SD)) {
    Serial.println(F("initialization failed!"));
    return;
  }
  Serial.println(F("initialization done."));
}

void loop () {

  RtcDateTime now = Rtc.GetDateTime();

  

  /*  if ( BTserial.available() > 0) {
      datasig = BTserial.read();
      Serial.println(F("블루투스데이터받음"));
      Serial.println(datasig);
      delay(1);
    } */
  if (BTserial.available() > 0) {
    datasig = BTserial.read();
  }

  if (datasig == 'o') {
    digitalWrite(CS_SD, LOW);
    // Open file

    sd_read();
   
    while (myFile.available()) {
     sddata = myFile.read();
     Serial.write(sddata);
      BTserial.print(sddata);
      datasig = 0;
    }
    
    myFile.close();
    
    // Disables SD card chip select pin
    digitalWrite(CS_SD, HIGH);
  }
  
   if (rfid.PICC_IsNewCardPresent() && RFID_ONE == 0) {
    readRFID();
    RFID_ONE = 1;
  }

  //운동 시작시 초기에 한번만 실행되는 조건문
  if (c == 0 && cnt > 1 && cnt != cnt2) {


    c = 1;
    cnt = cnt_jump;
    t = 0.01 ;     ////////////////////////////////////////////////////////////시간
    index = 0.00;    ////////////////////////////////////////////////////////////인덱스
  }

  else if (c == 1 && cnt > 0 && cnt != cnt2) {

    distance_add = (distance_add + distance);
    spd_add = (spd_add + spd);
    av_spd = (spd_add / index);
    BPM_add = (BPM_add + BPM);
    BPM_av = (BPM_add / int(index));
    int(kcal) = t * ((mets * weight * 3.5) / 1200);


    cnt_jump = cnt;

    lcd.setCursor(0, 0);            // 1번째, 1라인
    printDateTime_lcd(now);
 //   lcd.setCursor(11, 0);
 //   lcd.print("BPM:");
 //   lcd.setCursor(15, 0);
 //   lcd.print(BPM);
    lcd.setCursor(0, 1);            // 1번째, 2라인
    lcd.print("Time:");
    lcd.setCursor(5, 1);
    lcd.print(int(t));
    lcd.setCursor(11, 1);
    lcd.print("BPM:");
    lcd.setCursor(15, 1);
    lcd.print(BPM);
    lcd.setCursor(0, 2);            // 4번째, 3라인
    lcd.print("KCAL:");
    lcd.setCursor(5, 2);            // 4번째, 3라인
    lcd.print(kcal);
    lcd.setCursor(11, 2);
    lcd.print("SPD:");
    lcd.setCursor(15, 2);
    lcd.print(spd);
    lcd.setCursor(0, 3);            // 9번째, 4라인
    lcd.print("DIST:");
    lcd.setCursor(5, 3);
    lcd.print(distance_add );
    lcd.setCursor(11, 3);


    printDateTime(now);  //이게 시간그거
//    Serial.print(",");
//    Serial.print(index);
//    Serial.print(",");
//    Serial.println(t);
//    Serial.print("," );
//    Serial.print(distance_add / 1000);
//    Serial.print("," );
//    Serial.print(av_spd);
 //   Serial.print(",");
//    Serial.print(BPM_av);
 //   Serial.print("," );
//    Serial.println(kcal);




    cnt = 1;
    cnt2 = cnt;
    a = 0;

  }

  else if (cnt > 0 && cnt2 == cnt && a == 0) {


    // Enables SD card chip select pin
    digitalWrite(CS_SD, LOW);
    
    // Open file
    sd_write();


    printDateTime(now); 
    Serial.print(F("/"));
    Serial.print(id);
    Serial.print(F("/"));
    Serial.print(int(t));
    Serial.print(F("/"));
    Serial.print(int(distance_add*100)); // 
    Serial.print(F("/"));
    Serial.print(int(av_spd*100));
    Serial.print(F("/"));
    Serial.print(BPM_av);
    Serial.print(F("/"));
    Serial.print(kcal);


    printDateTime_sd(now);
    myFile.print(id);
    myFile.print("/");       
    time_sd();
    myFile.print("/");
    myFile.print(int(distance_add*100)); // *100
    myFile.print("/");
    myFile.print(int(av_spd*100)); // *100
    myFile.print("/");
    bpm();
    myFile.print("/");
    myFile.println(int(kcal)); // *100




    myFile.close();

    // Disables SD card chip select pin
    digitalWrite(CS_SD, HIGH);


    a = 1;

    //초기값
    cnt = 0;
    cnt2 = 2;
    cnt_jump = 0;
    a = 0;
    c = 0;
    index = 1;
    t = 0.0f;
    distance_add = 0.0f;
    kcal = 0.0f;
    RFID_ONE = 0;


  }
  // 파일을 닫아준다.

  delay(1909);
}


//RTC관련
#define countof(a) (sizeof(a) / sizeof(a[0]))

void printDateTime(const RtcDateTime& dt)
{
  char datestring[20];

  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u/%02u/%04u %02u:%02u:%02u"),
             dt.Month(),
             dt.Day(),
             dt.Year(),
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
             PSTR("%04u%02u%02u%02u%02u"),
             dt.Year(),
             dt.Month(),
             dt.Day(),
             dt.Hour(),
             dt.Minute()
            );
  myFile.print(datestring);
}

/*void printDateTime_blu(const RtcDateTime& dt)
  {
  char datestring[20];

  snprintf_P(datestring,
             countof(datestring),
             PSTR("%04u%02u%02u%02u%02u"),
             dt.Year(),
             dt.Month(),
             dt.Day(),
             dt.Hour(),
             dt.Minute());
  BTserial.print(datestring);
  }  */

void printDateTime_lcd(const RtcDateTime& dt)
{
  char datestring[20];

  snprintf_P(datestring,
             countof(datestring),
             PSTR("%02u%02u %02u:%02u"),
             dt.Month(),
             dt.Day(),
             dt.Hour(),
             dt.Minute() );
  lcd.print(datestring);
}
