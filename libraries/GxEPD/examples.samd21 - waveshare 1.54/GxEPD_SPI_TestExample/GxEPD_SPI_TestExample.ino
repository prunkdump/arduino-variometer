/************************************************************************************
   GxEPD_TestExample : test example for e-Paper displays from GoodDisplay.com

   based on Demo Example from GoodDisplay.com, avalable with any order for such a display, no copyright notice.

   Author : J-M Zingg

   modified by :

   Version : 1.1

   Support: minimal, provided as example only, as is, no claim to be fit for serious use

   connection to the e-Paper display is through DESTM32-S2 connection board, available from GoodDisplay

   DESTM32-S2 pinout (top, component side view):
       |-------------------------------------------------
       |  VCC  |o o| VCC 5V
       |  GND  |o o| GND
       |  3.3  |o o| 3.3V
       |  nc   |o o| nc
       |  nc   |o o| nc
       |  nc   |o o| nc
       |  MOSI |o o| CLK
       |  DC   |o o| D/C
       |  RST  |o o| BUSY
       |  nc   |o o| BS
       |-------------------------------------------------
*/

#include <SDU.h> 

// include library, include base class, make path known
#include <GxEPD.h>

// select the display class to use, only one
#include <GxGDEP015OC1/GxGDEP015OC1.cpp>
//#include <GxGDE0213B1/GxGDE0213B1.cpp>
//#include <GxGDEH029A1/GxGDEH029A1.cpp>
//#include <GxGDEW027C44/GxGDEW027C44.cpp>
//#include <GxGDEW042T2/GxGDEW042T2.cpp>
//#include <GxGDEW075T8/GxGDEW075T8.cpp>

// uncomment next line for drawBitmap() test, (consumes RAM on ESP8266)
#include GxEPD_BitmapExamples

// FreeFonts from Adafruit_GFX
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>

#include <GxIO/GxIO_SPI/GxIO_SPI.cpp>
#include <GxIO/GxIO.cpp>

/*#if defined(ESP8266)
//GxIO_SPI(SPIClass& spi, int8_t cs, int8_t dc, int8_t rst = -1, int8_t bl = -1);
GxIO_Class io(SPI, SS, D3, D4);
#else

// E-Ink   Mkrzero
// CS      D4
// BUSY    D5
// RST     D6
// DC      D7
// DIN     MOSI/D8
// CLK     SCK/D9
// GND     GNG
// 3.3V    3.3V

// BUSY->D5, RST->D6, DC->D7, CS->D4, CLK->D9(SCK), DIN->D8(MOSI), GND->GNG, 3.3V->3.3V


//GxIO_Class io(SPI, SS, 8, 9);
GxIO_Class io(SPI, 4, 7, 6);
#endif

GxEPD_Class display(io,6,5);
*/

#if defined(ESP8266)

// generic/common.h
//static const uint8_t SS    = 15;
//static const uint8_t MOSI  = 13;
//static const uint8_t MISO  = 12;
//static const uint8_t SCK   = 14;
// pins_arduino.h
//static const uint8_t D8   = 15;
//static const uint8_t D7   = 13;
//static const uint8_t D6   = 12;
//static const uint8_t D5   = 14;

//GxIO_SPI(SPIClass& spi, int8_t cs, int8_t dc, int8_t rst = -1, int8_t bl = -1);
GxIO_Class io(SPI, SS, D3, D4);
// GxGDEP015OC1(GxIO& io, uint8_t rst = D4, uint8_t busy = D2);
GxEPD_Class display(io);

#elif defined(ESP32)

// pins_arduino.h, e.g. LOLIN32
//static const uint8_t SS    = 5;
//static const uint8_t MOSI  = 23;
//static const uint8_t MISO  = 19;
//static const uint8_t SCK   = 18;

// GxIO_SPI(SPIClass& spi, int8_t cs, int8_t dc, int8_t rst = -1, int8_t bl = -1);
GxIO_Class io(SPI, SS, 17, 16);
// GxGDEP015OC1(GxIO& io, uint8_t rst = D4, uint8_t busy = D2);
GxEPD_Class display(io, 16, 4);

#elif defined(ARDUINO_ARCH_SAMD)

// variant.h of MKR1000
//#define PIN_SPI_MISO  (10u)
//#define PIN_SPI_MOSI  (8u)
//#define PIN_SPI_SCK   (9u)
//#define PIN_SPI_SS    (24u) // should be 4?
// variant.h of MKRZERO
//#define PIN_SPI_MISO  (10u)
//#define PIN_SPI_MOSI  (8u)
//#define PIN_SPI_SCK   (9u)
//#define PIN_SPI_SS    (4u)

GxIO_Class io(SPI, 4, 7, 6);
GxEPD_Class display(io, 6, 5);

#elif defined(_BOARD_GENERIC_STM32F103C_H_)

// STM32 Boards (STM32duino.com)
// Generic STM32F103C series
// aka BluePill
// board.h
//#define BOARD_SPI1_NSS_PIN        PA4
//#define BOARD_SPI1_MOSI_PIN       PA7
//#define BOARD_SPI1_MISO_PIN       PA6
//#define BOARD_SPI1_SCK_PIN        PA5
//enum {
//    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13,PA14,PA15,
//  PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13,PB14,PB15,
//  PC13, PC14,PC15
//};
// variant.h
//static const uint8_t SS   = BOARD_SPI1_NSS_PIN;
//static const uint8_t SS1  = BOARD_SPI2_NSS_PIN;
//static const uint8_t MOSI = BOARD_SPI1_MOSI_PIN;
//static const uint8_t MISO = BOARD_SPI1_MISO_PIN;
//static const uint8_t SCK  = BOARD_SPI1_SCK_PIN;

//GxIO_SPI(SPIClass& spi, int8_t cs, int8_t dc, int8_t rst = -1, int8_t bl = -1);
GxIO_Class io(SPI, SS, 8, 9);
//  GxGDEP015OC1(GxIO& io, uint8_t rst = 9, uint8_t busy = 7);
GxEPD_Class display(io, 9, 3);

#else

// pins_arduino.h, e.g. AVR
#define PIN_SPI_SS    (10)
#define PIN_SPI_MOSI  (11)
#define PIN_SPI_MISO  (12)
#define PIN_SPI_SCK   (13)

GxIO_Class io(SPI, SS, 8, 9);
//GxIO_DESTM32L io;
//GxIO_GreenSTM32F103V io;
// GxGDEP015OC1(GxIO& io, uint8_t rst = 9, uint8_t busy = 7);
GxEPD_Class display(io);

#endif

/*#if defined(ARDUINO_SAMD)
  // Required for Serial on Zero based boards
  #define Serial SerialUSB
#endif*/


void setup()
{
/*  Serial.begin(9600);
  while (!Serial) { ;}*/

  Serial.println();
  Serial.println("setup");

  display.init();

  Serial.println("setup done");
}

void loop()
{
  showBitmapExample();
  showFont("FreeMonoBold9pt7b", &FreeMonoBold9pt7b);
  showFont("FreeMonoBold12pt7b", &FreeMonoBold12pt7b);
  showFont("FreeMonoBold18pt7b", &FreeMonoBold18pt7b);
  showFont("FreeMonoBold24pt7b", &FreeMonoBold24pt7b);
  delay(10000);
}

void showBitmapExample()
{
#ifdef _GxBitmapExamples_H_
#ifdef _GxGDEW027C44_H_
  // draw black and red bitmap
  display.drawPicture(BitmapExample1, BitmapExample2, sizeof(BitmapExample1));
  delay(2000);
  display.setRotation(3);
#else
  display.drawBitmap(BitmapExample1, sizeof(BitmapExample1));
  delay(2000);
  display.drawBitmap(BitmapExample2, sizeof(BitmapExample2));
  delay(2000);
  display.setRotation(2);
  display.fillScreen(GxEPD_WHITE);
  // bitmap may have been shortened to fit to available RAM
  uint16_t h = min(GxEPD_HEIGHT, sizeof(BitmapExample1) * 8 / GxEPD_WIDTH);
  //Serial.println(h);
  display.drawBitmap(0, 0, BitmapExample1, GxEPD_WIDTH, h, GxEPD_BLACK);
  display.update();
  delay(2000);
#endif
#endif
}

void showFont(const char name[], const GFXfont* f)
{
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(f);
  display.setRotation(4);
  display.setCursor(0, 0);
  display.println();
  display.println(name);
  display.println(" !\"#$%&'()*+,-./");
  display.println("0123456789:;<=>?");
  display.println("@ABCDEFGHIJKLMNO");
  display.println("PQRSTUVWXYZ[\\]^_");
#ifdef _GxGDEW027C44_H_
  display.setTextColor(GxEPD_RED);
#endif
  display.println("`abcdefghijklmno");
  display.println("pqrstuvwxyz{|}~ ");
  display.update();
  delay(5000);
}




