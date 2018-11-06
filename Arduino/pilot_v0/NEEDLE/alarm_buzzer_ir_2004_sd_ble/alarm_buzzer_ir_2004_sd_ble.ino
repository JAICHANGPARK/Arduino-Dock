////////////////////////////////////////////////////////////////
////     LCD   VCC->5V    GND->GND    SDA->A4   SCL->A5   //////
////     RTC   VCC->5V    GND->GND    SDA->A4   SCL->A5   //////
////     BUZZER      GND->GND    DIGITAL -> 9           //////
////     SW1   DIGITAL -> 2                               //////
////     SW2   DIGITAL -> 3                               //////
////     SW3   DIGITAL -> 5                               //////
//////   sd  sck->13  DO->12  DI->11 CS->4             ///////
//////   ir  out->8                                   ////////
//////   BT   TX->6  RX->10                              ////////
////////////////////////////////////////////////////////////////

#include <DS3231.h> //Memanggil RTC3231 Library
#include <Wire.h>  // i2C Conection Library
#include <LiquidCrystal_I2C.h>  //Meman ggil i2C LCD Library
#include <Button.h> //Memanggil library Push Button
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(6,10); // RX, TX

File myFile;
//Sd2Card card;
//SdVolume volume;
//SdFile root;
//int index=1;
//const int chipSelect = 4;

//Setup Tombol Setting
#define DN_PIN 2 //Decrease Button
#define UP_PIN 3 //Increase Button
//#define SET_PIN 4  //Setup Button
#define ALR_PIN 5  //Setup Button
#define PULLUP true //Mengaktifkan internal Pull Up
#define INVERT true 
#define DEBOUNCE_MS 20     
#define REPEAT_FIRST 500 
#define REPEAT_INCR 100 

//Declare push buttons
Button btnUP(UP_PIN, PULLUP, INVERT, DEBOUNCE_MS);    
Button btnDN(DN_PIN, PULLUP, INVERT, DEBOUNCE_MS);
//Button btnSET(SET_PIN, PULLUP, INVERT, DEBOUNCE_MS);
Button btnALR(ALR_PIN, PULLUP, INVERT, DEBOUNCE_MS);
enum {WAIT, INCR, DECR};             
uint8_t STATE;                       
//int count;                           
int lastCount = -1;                   
unsigned long rpt = REPEAT_FIRST;  

int buzzer = 9;

int needle=30;

int sensorpin = 8; // 센서값을 읽을 단자 설정
int val; // 센서값

LiquidCrystal_I2C lcd(0x27, 20, 4);
//custom degrees char
uint8_t dgrsChar[8] = {0x4,0xa,0x4,0x0,0x0,0x0,0x0,0x0}; 

// Init DS3231
DS3231  rtc(SDA, SCL);

// Init a Time-data structure
Time  t; //pencacah string time()

uint32_t targetTime = 0;
uint8_t conv2d(const char* p) {
uint8_t v = 0;
  if ('0' <= *p && *p <= '9')
   v = *p - '0';
  return 10 * v + *++p - '0';}

//Mengambil waktu jam dari waktu Compile/Upload di system Windows
uint8_t hh = 0, mm = 0, ss = 0, dd = 0, bb = 0;
int yy = 0;
String Day = "  ";
uint8_t alarmHH = 9, alarmMM = 30;
int alarmLONG = 5;
uint8_t setMode = 0, setAlarm = 0, alarmMode=0;

char Sread = Serial.read();

void setup()
{
  Serial.begin(9600);
  BTSerial.begin(9600);
  
  // Setup Serial connection
  //Serial.begin(115200);
  pinMode (13, OUTPUT); 
  //pinMode(push,  INPUT);
   pinMode (sensorpin, INPUT); // 센서값을 인풋으로 설정
    
  //digitalWrite (13, LOW); // jika menggunakan relay sebagai beban alarm
  lcd.init(); // memulai koneksi i2c dengan LCD
  rtc.begin(); // memulai koneksi i2c dengan RTC

  
  //SD카드 확인
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");

  Serial.print("needle=");
    Serial.println(needle);

    lcd.setCursor(3,3);
     lcd.print(F("Needle: "));
     lcd.print(needle);
//rtc.setTime (10, 15, 00); 
//rtc.setDate(23, 8, 2016);  
}

void countdown(){
  val = digitalRead (sensorpin); // 센서값을 읽어옴
  
  if (val == LOW) { // 장애물 감지 안됨
     needle=needle-1;
   
    Serial.print(yy);
     myFile.print(yy);
     if(bb<=9)
    {
      Serial.print("0");
      myFile.print("0");
    };
    Serial.print(bb);
    myFile.print(bb);
    
   if(dd<=9)
    {
      Serial.print("0");
      myFile.print("0");
    };
     Serial.print(dd);
     myFile.print(dd);
    
    if(hh<=9)
    {
      Serial.print("0");
       myFile.print("0");
    };
    Serial.print(hh);
    myFile.print(hh);
    
     if(mm<=9)
    {
      Serial.print("0");
      myFile.print("0");
    };
    Serial.print(mm);
    myFile.print(mm);

   Serial.print("needle=");
   Serial.println(needle);
   lcd.setCursor(3,3);
     lcd.print(F("Needle: "));
     if(needle<10){lcd.print(F("0"));}
     lcd.print(needle);

      delay(1000);
    }
}

void zero(){
    while (needle==0){
      lcd.noBacklight();
      delay(100);
      lcd.backlight();
      delay(100);

    if(digitalRead(UP_PIN)==LOW){
      lcd.backlight();
      needle=30;
    Serial.print("needle=");
   Serial.println(needle);
   
   lcd.setCursor(3,3);
     lcd.print(F("Needle: "));
     if(needle<10){lcd.print(F("0"));}
     lcd.print(needle);
     
  }
  }
}


void loop()
{
  
  t = rtc.getTime();
Day = rtc.getDOWStr(1);

 myFile = SD.open("test.txt", FILE_WRITE);

     countdown();   
  zero();  
    myFile.close();

if (setMode == 0){
hh = t.hour,DEC;
mm = t.min,DEC;
ss = t.sec,DEC;
dd = t.date,DEC;
bb = t.mon,DEC;
yy = t.year,DEC;
}  

  if (setAlarm <2){
  lcd.setCursor(3,0);
  
  lcd.print(yy);
  lcd.print(F("."));
  if (bb<10){ lcd.print(F("0"));}
  lcd.print(bb);
  lcd.print(F("."));
  if (dd<10){ lcd.print(F("0"));}
  lcd.print(dd);
  lcd.print(F("-"));
  if(setMode==0)lcd.print(Day); else {lcd.print(F("SET"));}
  
  lcd.setCursor(6,1);
  if (hh<10){ lcd.print(F("0"));}
  lcd.print(hh); lcd.print(F(":"));
  if (mm<10){ lcd.print(F("0"));} 
  lcd.print(mm); lcd.print(F(":"));
  if (ss<10){ lcd.print(F("0"));} 
  lcd.print(ss);

  lcd.setCursor(3,2); 
  lcd.print(F("ALARM "));
    
  if(alarmHH<10){lcd.print(F("0"));}
  lcd.print (alarmHH); lcd.print(F(":"));
  if(alarmMM<10){lcd.print(F("0"));}
  lcd.print (alarmMM);
  lcd.print(F(":00"));

  lcd.setCursor(3,3);
     lcd.print(F("Needle: "));
     if(needle<10){lcd.print(F("0"));}
     lcd.print(needle);

    
  }
  
  setupClock();
  Alarm (alarmHH, alarmMM, alarmLONG);

if (setAlarm <2 && setMode != 0) {delay (100);}
if (setAlarm <2 && setMode == 1){lcd.setCursor(5,0); lcd.print(F("  ")); delay(100);}
if (setAlarm <2 && setMode == 2 && bb>9){lcd.setCursor(8,0); lcd.print(F("  ")); delay(100);}
if (setAlarm <2 && setMode == 2 && bb<=9){lcd.setCursor(8,0); lcd.print(F(" ")); delay(100);}
if (setAlarm <2 && setMode == 3 && bb>9){lcd.setCursor(11,0); lcd.print(F("    ")); delay(100);}
if (setAlarm <2 && setMode == 3 && bb<=9){lcd.setCursor(10,0); lcd.print(F("    ")); delay(100);}
if (setAlarm <2 && setMode == 4){lcd.setCursor(0,1); lcd.print(F("  ")); delay(100);}
if (setAlarm <2 && setMode == 5){lcd.setCursor(3,1); lcd.print(F("  ")); delay(100);}
if (setAlarm <2 && setMode == 6){lcd.setCursor(6,1); lcd.print(F("  ")); delay(100);}

if (setMode == 0 && setAlarm != 0  && setAlarm != 1) {delay (100);}
//if (setMode == 0 && setAlarm == 1){lcd.clear(); lcd.setCursor(3,0); lcd.print(F("Alarm OFF")); delay(100);}
if (setMode == 0 && setAlarm == 2){lcd.setCursor(2,3); lcd.print(F("  ")); delay(100);}
if (setMode == 0 && setAlarm == 3){lcd.setCursor(5,3); lcd.print(F("  ")); delay(100);}
if (setMode == 0 && setAlarm == 4 && alarmLONG >9){lcd.setCursor(16,3); lcd.print(F("  ")); delay(100);}
if (setMode == 0 && setAlarm == 4 && alarmLONG <10){lcd.setCursor(16,3); lcd.print(F(" ")); delay(100);}

//Serial.print (setMode);
//Serial.print ("  ");
//Serial.println (setAlarm);

if (BTSerial.available() > 0) {
     Sread = BTSerial.read();
     Serial.println(Sread);
 }

//  if (Serial.available() > 0) {
//    Sread = Serial.read();
//    Serial.println(Sread);
//  }

 myFile = SD.open("test.txt");
 if (Sread == 'a')
 {
   while (myFile.available()) {
     char inChar = myFile.read();
     Serial.print(inChar);
     BTSerial.print(inChar);
     Sread = 0;
     //SD.remove("test.txt");
     //delay (10);
   }
 }
 if (Sread == 'b')
 {
   while (myFile.available()) {
     char inChar = myFile.read();
     Serial.print("1");
     BTSerial.print("1");
     Sread = 0;
     //SD.remove("test.txt");
     //delay (10);
     break;
   }
 }
 myFile.close();
}

void setupClock (void) {
   btnUP.read();                             //read the buttons
   btnDN.read();
   //btnSET.read();
   btnALR.read();
   
   /******************************************
    if (setMode == 7){
    lcd.setCursor (0,0);
    lcd.print (F("Set Date Finish "));
    lcd.setCursor (0,1);
    lcd.print (F("Set Time Finish "));delay (1000);
    rtc.setTime (hh, mm, ss);
    rtc.setDate (dd, bb, yy);  
    lcd.clear();
    setMode = 0;}
    *******************************************************/

    if (setAlarm == 5){
    lcd.setCursor (3,2);
    lcd.print (F("SET ALARM FINISH"));
    lcd.setCursor (2,3);
    lcd.print (alarmHH); lcd.print (F(":"));
    lcd.print (alarmMM); 
    lcd.print (F(":00"));
    delay (2000); lcd.clear();
    setAlarm=0;
    alarmMode=1;}
    
    if (setAlarm >0){ alarmMode=0;}
    
    switch (STATE) {
        
        case WAIT:                            
            //if (btnSET.wasPressed())
              // { setMode = setMode+1;}
            if (btnALR.wasPressed())
               { setAlarm = setAlarm+1;} 
            if (btnUP.wasPressed())
                STATE = INCR;
            else if (btnDN.wasPressed())
                STATE = DECR;
            else if (btnUP.wasReleased())    
                rpt = REPEAT_FIRST;
            else if (btnDN.wasReleased())
                rpt = REPEAT_FIRST;
            else if (btnUP.pressedFor(rpt)) {  
                rpt += REPEAT_INCR;            
                STATE = INCR;
            }
            else if (btnDN.pressedFor(rpt)) {
                rpt += REPEAT_INCR;
                STATE = DECR;
            }
            break;

        case INCR:                               
            if (setAlarm<2 && setMode==1 && dd<31)dd=dd+1; 
            if (setAlarm<2 && setMode==2 && bb<12)bb=bb+1;
            if (setAlarm<2 && setMode==3 && yy<2050)yy=yy+1;
            if (setAlarm<2 && setMode==4 && hh<23)hh=hh+1;
            if (setAlarm<2 && setMode==5 && mm<59)mm=mm+1;
            if (setAlarm<2 && setMode==6 && ss<59)ss=ss+1;
            if (setMode==0 && setAlarm==2 && alarmHH<23)alarmHH=alarmHH+1;
            if (setMode==0 && setAlarm==3 && alarmMM<59)alarmMM=alarmMM+1;
            if (setMode==0 && setAlarm==4 && alarmLONG<59)alarmLONG=alarmLONG+1;
            STATE = WAIT;
            break;

        case DECR:                               
            if (setAlarm<2 && setMode==1 && dd>0)dd=dd-1;
            if (setAlarm<2 && setMode==2 && bb>0)bb=bb-1;
            if (setAlarm<2 && setMode==3 && yy>2000)yy=yy-1;
            if (setAlarm<2 && setMode==4 && hh>0)hh=hh-1;
            if (setAlarm<2 && setMode==5 && mm>0)mm=mm-1;
            if (setAlarm<2 && setMode==6 && ss>0)ss=ss-1;  
            if (setMode==0 && setAlarm==2 && alarmHH>0)alarmHH=alarmHH-1;
            if (setMode==0 && setAlarm==3 && alarmMM>0)alarmMM=alarmMM-1;
            if (setMode==0 && setAlarm==4 && alarmLONG>0)alarmLONG=alarmLONG-1;
            STATE = WAIT;
            break;
    }
}


void Alarm (uint8_t alarmHH, uint8_t alarmMM,int alarmLONG) {

 if (alarmMode==1 && hh == alarmHH && (mm - alarmMM >= 0 ) && (mm - alarmMM <= alarmLONG )) {
   tone(buzzer, 330, 523);
   delay(50);
    tone(buzzer, 294, 523);
   delay(50);
    tone(buzzer, 261, 523);
   delay(50);
    tone(buzzer, 294, 523);
   delay(50);
    tone(buzzer, 330, 523);
   delay(50);
   tone(buzzer, 330, 523);
   delay(50);
   tone(buzzer, 330, 523);
   delay(50);
    tone(buzzer, 294, 523);
   delay(50);
    tone(buzzer, 294, 523);
   delay(50);
    tone(buzzer, 294, 523);
   delay(50);
    tone(buzzer, 330, 523);
   delay(50);
    tone(buzzer, 392, 523);
     delay(50);
     tone(buzzer, 392, 523);
     delay(50);
     tone(buzzer, 330, 523);
     delay(50);
     tone(buzzer, 294, 523);
     delay(50);
     tone(buzzer, 261, 523);
     delay(50);
     tone(buzzer, 294, 523);
     delay(50);
     tone(buzzer, 330, 523);
     delay(50);
     tone(buzzer, 330, 523);
     delay(50);
     tone(buzzer, 330, 523);
     delay(50);
     tone(buzzer, 294, 523);
     delay(50);
     tone(buzzer, 294, 523);
     delay(50);
     tone(buzzer, 330, 523);
     delay(50);
     tone(buzzer, 294, 523);
     delay(50);
     tone(buzzer, 261, 523);
     delay(50);
     tone(buzzer, 523, 523);
     delay(50);
  
  if(ss%2==0){lcd.noBacklight();}
  else {lcd.backlight ();}
  
  btnALR.read();
  if (btnALR.wasPressed())
 { alarmMode = 0; lcd.backlight();}}
 
 
 else {noTone (13);  lcd.backlight();}

 if (setMode == 0 && setAlarm !=0 && setAlarm !=1){
  lcd.setCursor(3,0);
  
  lcd.print(yy);
  lcd.print(F("."));
  if (bb<10){ lcd.print(F("0"));}
  lcd.print(bb);
  lcd.print(F("."));
  if (dd<10){ lcd.print(F("0"));}
  lcd.print(dd);
  lcd.print(F("-"));
  if(setMode==0)lcd.print(Day); else {lcd.print(F("SET"));}
  
  lcd.setCursor(6,1);
  if (hh<10){ lcd.print(F("0"));}
  lcd.print(hh); lcd.print(F(":"));
  if (mm<10){ lcd.print(F("0"));} 
  lcd.print(mm); lcd.print(F(":"));
  if (ss<10){ lcd.print(F("0"));} 
  lcd.print(ss);
  
  lcd.setCursor (3,2);
  lcd.print(F("   SET ALARM  "));
  lcd.setCursor (2,3);
  if(alarmHH<10){lcd.print(F("0"));}
  lcd.print (alarmHH); lcd.print(F(":"));
  if(alarmMM<10){lcd.print(F("0"));}
  lcd.print (alarmMM); 
  {lcd.print(F(":00 "));}
  lcd.print(F("Long:"));
  lcd.print (alarmLONG); lcd.print(F("MIN "));
 }
}


