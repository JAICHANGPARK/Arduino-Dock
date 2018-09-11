
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>


#define TFT_DC     7
#define TFT_RST    8  
#define TFT_CS     9

// For 1.44" and 1.8" TFT with ST7735 use
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

// For 1.54" TFT with ST7789
//Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS,  TFT_DC, TFT_RST);

// Option 2: use any pins but a little slower!
//#define TFT_SCLK 13   // set these to be whatever pins you like!
//#define TFT_MOSI 11   // set these to be whatever pins you like!
//Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);


float p = 3.1415926;
long tftTimeIndex = 0;
int lcdCnt = 0;
void setup(void) {
  Serial.begin(9600);
  Serial.print("Hello! ST77xx TFT Test");

  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

  Serial.println("Initialized");
  uint16_t time = millis();
  tft.fillScreen(ST77XX_BLACK);
  time = millis() - time;

  Serial.println(time, DEC);
  delay(500);

  // large block of text
  tft.fillScreen(ST77XX_BLACK);
  testdrawtext("Lorem ipsum dolor sit amet, consectetur adipiscing elit. Curabitur adipiscing ante sed nibh tincidunt feugiat. Maecenas enim massa, fringilla sed malesuada et, malesuada sit amet turpis. Sed porttitor neque ut ante pretium vitae malesuada nunc bibendum. Nullam aliquet ultrices massa eu hendrerit. Ut sed nisi lorem. In vestibulum purus a tortor imperdiet posuere. ", ST77XX_WHITE);
  delay(1000);

  tftPrintTest2();
  delay(4000);

  // tft print function!
  tftPrintTest();
  delay(4000);

  //  // a single pixel
  //  tft.drawPixel(tft.width()/2, tft.height()/2, ST77XX_GREEN);
  //  delay(500);
  //
  //  // line draw test
  //  testlines(ST77XX_YELLOW);
  //  delay(500);
  //
  //  // optimized lines
  //  testfastlines(ST77XX_RED, ST77XX_BLUE);
  //  delay(500);
  //
  //  testdrawrects(ST77XX_GREEN);
  //  delay(500);
  //
  //  testfillrects(ST77XX_YELLOW, ST77XX_MAGENTA);
  //  delay(500);
  //
  //  tft.fillScreen(ST77XX_BLACK);
  //  testfillcircles(10, ST77XX_BLUE);
  //  testdrawcircles(10, ST77XX_WHITE);
  //  delay(500);
  //
  //  testroundrects();
  //  delay(500);
  //
  //  testtriangles();
  //  delay(500);
  //
  //  mediabuttons();
  //  delay(500);
  //
  //  Serial.println("done");
  //  delay(1000);
}

void loop() {

  int lcdCount = 4;
  long currerntMillis = millis();
  if (currerntMillis - tftTimeIndex >= 3000) {
    int lcdModulo = lcdCnt % 4 ;

    switch (lcdModulo) {
      case 0:
        tftPrintTest3();
        break;
      case 1:
        tftPrintTest4();
        break;
      case 2:
        tftPrintTest2();
        break;
      case 3:
        tftPrintTest3();
        break;
    }
    lcdCnt++;

    tft.invertDisplay(true);
    delay(500);
    tft.invertDisplay(false);
    delay(500);
    tftTimeIndex = millis();
  }

}


void testdrawtext(char *text, uint16_t color) {
  tft.setCursor(0, 0);
  tft.setTextColor(color);
  tft.setTextWrap(true);
  tft.print(text);
}


void tftPrintTest2() {
  tft.setTextWrap(true);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, 10);
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(2);
  tft.println("Distance");
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);
  tft.println("1.6 km");
}

void tftPrintTest3() {
  tft.setTextWrap(true);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, 10);
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(2);
  tft.println("HR");
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);
  tft.println("89 bpm");
}

void tftPrintTest4() {
  tft.setTextWrap(true);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(10, 10);
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(2);
  tft.println("Speed");
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(2);
  tft.println("5.0 km/h");
}


void tftPrintTest() {
  tft.setTextWrap(true);
  tft.fillScreen(ST77XX_BLACK);
  tft.setCursor(0, 30);
  tft.setTextColor(ST77XX_RED);
  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ST77XX_YELLOW);
  tft.setTextSize(2);
  tft.println("Hello World!");
  tft.setTextColor(ST77XX_GREEN);
  tft.setTextSize(3);
  tft.println("Hello World!");
  tft.setTextColor(ST77XX_BLUE);
  tft.setTextSize(4);
  tft.print(1234.567);
  delay(1500);
  tft.setCursor(0, 0);
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextColor(ST77XX_WHITE);
  tft.setTextSize(0);
  tft.println("Hello World!");
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_GREEN);
  tft.print(p, 6);
  tft.println(" Want pi?");
  tft.println(" ");
  tft.print(8675309, HEX); // print 8,675,309 out in HEX!
  tft.println(" Print HEX!");
  tft.println(" ");
  tft.setTextColor(ST77XX_WHITE);
  tft.println("Sketch has been");
  tft.println("running for: ");
  tft.setTextColor(ST77XX_MAGENTA);
  tft.print(millis() / 1000);
  tft.setTextColor(ST77XX_WHITE);
  tft.print(" seconds.");
}

void mediabuttons() {
  // play
  tft.fillScreen(ST77XX_BLACK);
  tft.fillRoundRect(25, 10, 78, 60, 8, ST77XX_WHITE);
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_RED);
  delay(500);
  // pause
  tft.fillRoundRect(25, 90, 78, 60, 8, ST77XX_WHITE);
  tft.fillRoundRect(39, 98, 20, 45, 5, ST77XX_GREEN);
  tft.fillRoundRect(69, 98, 20, 45, 5, ST77XX_GREEN);
  delay(500);
  // play color
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_BLUE);
  delay(50);
  // pause color
  tft.fillRoundRect(39, 98, 20, 45, 5, ST77XX_RED);
  tft.fillRoundRect(69, 98, 20, 45, 5, ST77XX_RED);
  // play color
  tft.fillTriangle(42, 20, 42, 60, 90, 40, ST77XX_GREEN);
}
