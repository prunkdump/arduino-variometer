// CJMCU-117 MPU9250+MS5611 circuit interface
//
// VCC  VCC - BAT
// GND  GND
// SCL  D12 - SCL
// SDA  D11 - SDA
//
// LM9110 
// VCC   VCC - BAT
// PWM   A3, A4 PWM
//
// A1,A2 Switch 
// A5    Detection ON/OFF  
//
// A6    Detection de connection USB
// D6    Commande de l'alimentation des cartes
// D0    Reset command
// 
// E-Ink
// VCC   3.3V M0
// CS    D1
// BUSY  D3
// RST   D2
// DC    D7
// DIN   MOSI/D8
// CLK   SCK/D9
//
// GPS 
// VCC   VCC - BAT
// TX    D5 serialNmea  Pin 5     Sercom4
// RX    TX Serial1     Pin 14    Sercom5
//
// Bluetooth 
// VCC   VCC - BAT
// TX    RX Serial1      Pin 13   Sercom5
// RX    D4 serialNmea   Pin 4    Sercom4

//SERCOM 0 WIRE / I2C - SDA/SCL     - MPU
//SERCOM 1 SPI  - MISO-MOSI-SCK     - SCREEN
//SERCOM 2 SPI SD                   - SDCARD
//SERCOM 3 
//SERCOM 4 UART                     - GPS / BT
//SERCOM 5 UART   - SERIAL1         - GPS / BT

#include <SDU.h>    //FIRMWARE Update

#include <Arduino.h>
#include <SPI.h>
#include <VarioSettings.h>

/*******************/
/* Version         */
/*******************/

#define VERSION 63
#define SUB_VERSION 0

/*******************/
/*     VERSION     */
/*    M0/SAMD21    */
/*                 */
/*    Historique   */
/*******************/
/* v 63.0     beta 1 version
 * 
 * v 63.0     beta 2 version
 *            version basée sur le code et les librairies M0 de PRUNKDUMP
 *            -Paramettres dans fichier TXT
 *            - Ecran I-Ink
 * v 63.0     beta 2.1 version           
 *            - add GPS Neo 8 
 *            - fix display bugs
 *            - add record Indicator, gps indicator
 * v 63.0     beta 2.2 version           
 *            - update display library
*******************/

/*****************/
/* screen        */
/*****************/
#ifdef HAVE_SCREEN

// include library, include base class, make path known
#include <GxEPD.h>

// select the display class to use, only one
#include <GxGDEP015OC1/GxGDEP015OC1NL.cpp>
//#include <GxGDE0213B1/GxGDE0213B1.cpp>
//#include <GxGDEH029A1/GxGDEH029A1.cpp>
//#include <GxGDEW042T2/GxGDEW042T2.cpp>

#include <GxIO/GxIO_SPI/GxIO_SPI.cpp>
#include <GxIO/GxIO.cpp>

// FreeFonts from Adafruit_GFX
/*#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

#include GxEPD_BitmapExamples*/

#include <gnuvario.h>

#include <varioscreen.h>

#if defined(VARIOSCREEN_BUSY_PIN)
GxIO_Class io(SPI, VARIOSCREEN_CS_PIN, VARIOSCREEN_DC_PIN, VARIOSCREEN_RST_PIN);
//GxEPD_Class display(io, VARIOSCREEN_RST_PIN, VARIOSCREEN_BUSY_PIN);
//VarioScreen display(io, VARIOSCREEN_RST_PIN, VARIOSCREEN_BUSY_PIN);
VarioScreen screen(io, VARIOSCREEN_RST_PIN, VARIOSCREEN_BUSY_PIN);
#else
GxIO_Class io(SPI, VARIOSCREEN_CS_PIN, VARIOSCREEN_DC_PIN, VARIOSCREEN_RST_PIN);
//GxEPD_Class display(io);
//VarioScreen display(io);
VarioScreen screen(io);
#endif //VARIOSCREEN_BUSY_PIN

#endif //HAVESCREEN

VarioSettings GnuSettings;

/*-----------------*/
/*      SETUP      */
/*-----------------*/
void setup() {

/************************/
/*        Init Power    */
/************************/

 #ifdef PROG_DEBUG
  char tmpbuffer[50];

  Serial.begin(9600);
  while (!Serial) { ;}
  sprintf(tmpbuffer,"SAMD21 MPU9250 MS5611 VARIO compiled on %s at %s", __DATE__, __TIME__);
  Serial.println(tmpbuffer);
  Serial.flush();
#endif //PRO_DEBBUG

 
  /***************/
  /* init screen */
  /***************/
#ifdef PROG_DEBUG
      Serial.println("initialization screen");
#endif //IMU_DEBUG

#ifdef HAVE_SCREEN
  screen.begin();

/*----------------------------------------*/
/*                                        */
/*             DISPLAY BOOT               */
/*                                        */
/*----------------------------------------*/

#ifdef PROG_DEBUG
  Serial.println("Display boot");
#endif //PRO_DEBBUG

  screen.fillScreen(GxEPD_WHITE);

  screen.drawBitmap(logo_gnuvario, 0, 10, 102, 74, GxEPD_BLACK); //94

  screen.setFont(&FreeSansBold12pt7b);
  screen.setTextSize(1);

  screen.setCursor(100, 30);
  screen.println("Version");
  screen.setCursor(105, 50);
  screen.println(" Beta 2");
  sprintf(tmpbuffer,"%02d.%02d", VERSION, SUB_VERSION);
  screen.setCursor(125, 70);
  screen.println(tmpbuffer);
  sprintf(tmpbuffer,"%s", __DATE__);
  screen.setCursor(25, 110);
  screen.println(tmpbuffer);

//  screen.update();
  screen.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);
  while (screen.GetState() != STATE_OK) {
    screen.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);
  }
#ifdef PROG_DEBUG
    Serial.print("update screen");
#endif //PRO_DEBBUG
  
#endif //HAVE_SCREEN

}


/*----------------*/
/*      LOOP      */
/*----------------*/
void loop() {
 }


