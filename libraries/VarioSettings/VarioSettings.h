#ifndef VARIO_SETTINGS_H
#define VARIO_SETTINGS_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FlashStorage.h>


/*----------------------------*/
/*          SOFTWARE          */
/*      Vario parameters      */
/*                            */
/*----------------------------*/

#define VARIOMETER_MODEL "GNUVario"


/*****************************/
/* SDCard/Bluetooth behavior */
/*****************************/

/* What type of barometric altitude is sent :           */
/* -> Based on international standard atmosphere        */
/* -> Calibrated with GPS altitude                      */
//#define VARIOMETER_SDCARD_SEND_CALIBRATED_ALTITUDE
//#define VARIOMETER_BLUETOOTH_SEND_CALIBRATED_ALTITUDE


/* When there is no GPS to sync variometer bluetooth sentences */
/* set the delay between sendings in milliseconds.             */ 
#define VARIOMETER_SENTENCE_DELAY 2000


/*----------------------------*/
/*          HARDWARE          */
/*      Vario parameters      */
/*                            */
/*----------------------------*/

/* Comment or uncomment according to  */
/* what you embed in the variometer   */ 
#define HAVE_SPEAKER
//#define HAVE_ACCELEROMETER
#define HAVE_SCREEN
#define HAVE_GPS
#define HAVE_SDCARD
#define HAVE_BLUETOOTH
#define HAVE_VOLTAGE_DIVISOR

// CJMCU-117 MPU9250+MS5611 circuit interface
//
// VCC  VCC
// GND  GND
// SCL  D12 - SCL
// SDA  D11 - SDA
//
// PWM   A3, A4 PWM
//
// A1,A2 Switch
// D1    Detection ON/OFF
//
// D0    Detection de connection USB
// A5    Commande de l'alimentation des cartes
// 
// E-Ink
// CS    D4
// BUSY  D5
// RST   D6
// DC    D7
// DIN   MOSI/D8
// CLK   SCK/D9
//
// GPS
// TX    D3 serialNmea  Pin 3
// RX    TX Serial1     Pin 14
//
// Bluetooth
// TX    RX Serial1      Pin 13
// RX    D2 serialNmea   Pin 2

/* Set the pins used for Screen modules */


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
//GxIO_Class io(SPI, SS, D3, D4);
// GxGDEP015OC1(GxIO& io, uint8_t rst = D4, uint8_t busy = D2);
//GxEPD_Class display(io);
// or my IoT connection, busy on MISO
//GxEPD_Class display(io, D4, D6);

#define VARIOSCREEN_CS_PIN SS
#define VARIOSCREEN_DC_PIN D3
#define VARIOSCREEN_RST_PIN D4


#elif defined(ESP32)

// pins_arduino.h, e.g. LOLIN32
//static const uint8_t SS    = 5;
//static const uint8_t MOSI  = 23;
//static const uint8_t MISO  = 19;
//static const uint8_t SCK   = 18;

//GxIO_Class io(SPI, SS, 17, 16);
//GxEPD_Class display(io, 16, 4);

#define VARIOSCREEN_CS_PIN SS
#define VARIOSCREEN_DC_PIN 17
#define VARIOSCREEN_RST_PIN 16
#define VARIOSCREEN_BUSY_PIN 4

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

//GxIO_Class io(SPI, 4, 7, 6);
//GxEPD_Class display(io, 6, 5);

#define VARIOSCREEN_CS_PIN 4
#define VARIOSCREEN_DC_PIN 7
#define VARIOSCREEN_RST_PIN 6
#define VARIOSCREEN_BUSY_PIN 5

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
//GxIO_Class io(SPI, SS, 8, 9);
//  GxGDEP015OC1(GxIO& io, uint8_t rst = 9, uint8_t busy = 7);
//GxEPD_Class display(io, 9, 3);
#define VARIOSCREEN_CS_PIN SS
#define VARIOSCREEN_DC_PIN 8
#define VARIOSCREEN_RST_PIN 9
#define VARIOSCREEN_BUSY_PIN 3

#else

// pins_arduino.h, e.g. AVR
//#define PIN_SPI_SS    (10)
//#define PIN_SPI_MOSI  (11)
//#define PIN_SPI_MISO  (12)
//#define PIN_SPI_SCK   (13)

//GxIO_Class io(SPI, SS, 8, 9);
//GxIO_DESTM32L io;
//GxIO_GreenSTM32F103V io;
//GxEPD_Class display(io);

#define VARIOSCREEN_CS_PIN SS
#define VARIOSCREEN_DC_PIN 8
#define VARIOSCREEN_RST_PIN 9

#endif

#define SDCARD_CS_PIN SDCARD_SS_PIN

/*int pinSDA = 11;
int pinSCL = 12;*/
//#define VARIODRDY_INT_PIN 3
//#define VARIOAUDIO_PWM1_PIN A3
//#define VARIOAUDIO_PWM2_PIN A4
#define VARIOBTN_CALIB_PIN A1
#define VARIOBTN_LEFT_PIN A1
#define VARIOBTN_RIGHT_PIN A2

/*interrupts in the Zero variant:

EXTERNAL_INT_2: A0, A5, 10
EXTERNAL_INT_4: A3, 6
EXTERNAL_INT_5: A4, 7
EXTERNAL_INT_6: 8, SDA
EXTERNAL_INT_7: 9, SCL
EXTERNAL_INT_9: A2, 3
EXTERNAL_INT_10: TX, MOSI
EXTERNAL_INT_11: RX, SCK*/

#define VARIOPOWER_INT_PIN 1
//const byte interruptPin = A5;

#define VARIO_DETECT_USB A6
#define VARIO_PIN_ALIM   A5
// A6    Detection de connection USB
// A5    Commande de l'alimentation des cartes


/* The voltage divisor */
#define VOLTAGE_DIVISOR_PIN ADC_BATTERY
#define VOLTAGE_DIVISOR_VALUE 1.27
#define VOLTAGE_DIVISOR_REF_VOLTAGE 3.3

/* The bauds rate used by the GPS and Bluetooth modules. */
/* GPS and bluetooth need to have the same bauds rate.   */
#define GPS_BLUETOOTH_BAUDS 9600

/* What type of vario NMEA sentence is sent by bluetooth. */
/* Possible values are :                                  */
/*  - VARIOMETER_SENT_LXNAV_SENTENCE                      */
/*  - VARIOMETER_SENT_LK8000_SENTENCE                     */
#define VARIOMETER_SENT_LK8000_SENTENCE

/********************/
/* Measure behavior */
/********************/


/* Speed filtering :                                               */
/* Greater values give smoother speed. The base unit is 2 seconds  */
/* so size = 5 use the last 10 seconds to average speed.           */
#define VARIOMETER_SPEED_FILTER_SIZE 5

/* Set the GPS precision needed to use the GPS altitude value  */
/* to calibrate the barometric altitude.                       */
/*      !!! the best possible precision is 100 !!!             */ 
#define VARIOMETER_GPS_ALTI_CALIBRATION_PRECISION_THRESHOLD 200

/* SPI speed                                 */
/* The variometer seems to be more stable at */
/* half speed. Don't hesitate to experiment. */
/*#if F_CPU >= 16000000L
#define VARIOSCREEN_SPEED SPI_CLOCK_DIV4
#define SDCARD_SPEED SPI_CLOCK_DIV4
#else
#define VARIOSCREEN_SPEED SPI_CLOCK_DIV2
#define SDCARD_SPEED SPI_CLOCK_DIV2
#endif //CPU_FREQ*/

// print useful information to the serial port for 
// verifying correct operation. Comment out to prevent
// data being spewed out continuously.

//              DEBUGING MODE
#define IMU_DEBUG			  //debug IMU
#define PROG_DEBUG			  //debug principal program
#define I2CDEV_SERIAL_DEBUG   //debug I2Cdev

//Monitor Port 
#define SerialPort SerialUSB


/******************************************************/
/******************************************************/


class VarioSettings {

 public:
  boolean initSettings();
  boolean readSDSettings();
  void writeSDSettings();
  uint8_t soundSettingRead(void);
  void soundSettingWrite(uint8_t volume);

#ifdef IMU_DEBUG 
  int exINT = 15;
  float exFloat = 1.12345;
  boolean exBoolean = true;
  long exLong = 2123456789;
#endif //IMU_DEBUG
  
  String VARIOMETER_PILOT_NAME = "Magali";
  String VARIOMETER_GLIDER_NAME = "MAC-PARA Muse 3";
  
  /* time zone relative to UTC */
  int8_t VARIOMETER_TIME_ZONE = (+2); 

  /*******************/
/* Screen behavior */
/*******************/

/* the duration of the two screen pages in milliseconds */
  int16_t VARIOMETER_BASE_PAGE_DURATION = 3000;
  int16_t VARIOMETER_ALTERNATE_PAGE_DURATION =3000;

  /*********/
  /* Beeps */
  /*********/

  /* The volume of the beeps, max = 10 */
  uint8_t VARIOMETER_BEEP_VOLUME = 3;

  /* The variometer react like this according to vertical speed in m/s :        */
  /* (near climbing beep is not enabled by default)                             */
  /*                                                                            */
  /* <--LOW-BEEP--|------SILENT------|--NEAR-CLIMBING-BEEP--|--CLIMBING-BEEP--> */
  /*              |                  |                      |                   */
  /*           SINKING         CLIMBING-SENSITIVITY      CLIMBING               */
  float VARIOMETER_SINKING_THRESHOLD =-2.0;
  float VARIOMETER_CLIMBING_THRESHOLD=0.2;
  float VARIOMETER_NEAR_CLIMBING_SENSITIVITY=0.5; 
  
  /* The near climbing alarm : signal that you enter or exit the near climbing zone */
  /* The near climbing beep : beep when you are in near climbing zone               */
  boolean VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM = false;
  boolean VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP  = false;

  /********************/
  /* Measure behavior */
  /********************/

  /* Flight start detection conditions :                      */
  /* -> Minimum time after poweron in milliseconds            */
  /* -> Minimum vertical velocity in m/s (low/high threshold) */
  /* -> Minimum ground speed in km/h                          */
  long FLIGHT_START_MIN_TIMESTAMP = 15000;
  float FLIGHT_START_VARIO_LOW_THRESHOLD = (-0.5);
  float FLIGHT_START_VARIO_HIGH_THRESHOLD = 0.5;
  float FLIGHT_START_MIN_SPEED = 8.0;

  /* GPS track recording on SD card starting condition :  */ 
  /* -> As soon as possible (GPS fix)                     */
  /* -> When flight start is detected                     */
   boolean VARIOMETER_RECORD_WHEN_FLIGHT_START = true;

  /* What type of vario NMEA sentence is sent by bluetooth. */
  /* Possible values are :                                  */
  /*  - VARIOMETER_SENT_LXNAV_SENTENCE                      */
  /*  - VARIOMETER_SENT_LK8000_SENTENCE                     */
  //boolean VARIOMETER_SENT_LXNAV_SENTENCE = true;

  /* Alarm */
  /* Alarm SDCARD not insert */
  boolean ALARM_SDCARD = true;
  /* Alarm GPS Fix */
  boolean ALARM_GPSFIX = true;
  /* Alarm Fly begin */
  boolean ALARM_FLYBEGIN = true;

// Kalman filter configuration
  float KF_ZMEAS_VARIANCE  =     400.0f;
  float KF_ZACCEL_VARIANCE =     1000.0f;
  float KF_ACCELBIAS_VARIANCE   = 1.0f;

// Power-down timeout. Here we power down if the
// vario does not see any climb or sink rate more than
// 50cm/sec, for 20 minutes.
   uint16_t SLEEP_TIMEOUT_SECONDS   = 1200; // 20 minutes
   uint8_t  SLEEP_THRESHOLD_CPS		= 50;

// vario thresholds in cm/sec for generating different
// audio tones. Between the sink threshold and the zero threshold,
// the vario is quiet
/*                                                                            */
/* <--LOW-BEEP--|------SILENT------|--NEAR-CLIMBING-BEEP--|--CLIMBING-BEEP--> */
/*              |                  |                      |                   */
/*             SINK              ZERO                   CLIMB                 */
   uint8_t CLIMB_THRESHOLD   =   50;
   int8_t ZERO_THRESHOLD	 =    5;
   int16_t SINK_THRESHOLD    =   -250;

// change these parameters based on the frequency bandwidth of the speaker

    uint16_t VARIO_MAX_FREQHZ   =   4000;
    uint16_t VARIO_XOVER_FREQHZ =   2000;
    uint16_t VARIO_MIN_FREQHZ   =   200;

    uint16_t VARIO_SINK_FREQHZ  =   400;
    uint16_t VARIO_TICK_FREQHZ  =   200;

// audio feedback tones
    uint16_t BATTERY_TONE_FREQHZ	=	400;
    uint16_t CALIB_TONE_FREQHZ		=	800;
    uint16_t MPU9250_ERROR_TONE_FREQHZ	= 200;
    uint16_t MS5611_ERROR_TONE_FREQHZ	= 2500;
    uint16_t SDCARD_ERROR_TONE_FREQHZ	= 2000;  
	uint16_t BEEP_FREQ                  = 800;
  
 protected:
  File myFile;
  char FileName[15] = "SETTINGS.TXT";
  
  void applySetting(String settingName, String settingValue);
  float toFloat(String settingValue);
  long toLong(String settingValue);
  boolean toBoolean(String settingValue);
};

extern VarioSettings GnuSettings;

class Statistic {

 public:
   void setTime(int8_t* timeValue);
  private:
    int8_t time[3];

};

#endif
