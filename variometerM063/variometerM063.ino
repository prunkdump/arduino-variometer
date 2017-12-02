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
#include <GenClock_zero.h>
#include <Wire.h>
#include <ms5611_zero.h>
#include <vertaccel.h>
#include <LightInvensense.h>
#include <kalmanvert.h>
#include <beeper.h>
#include <toneAC_zero.h>
#include <avr/pgmspace.h>
#include <digit.h>
#include <SD.h>
#include <SerialNmea_zero.h>
#include <NmeaParser.h>
#include <LxnavSentence.h>
#include <LK8Sentence.h>
#include <IGCSentence.h>

#include <accelcalibrator.h>

#include <Utility.h>

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


/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*!!            !!! WARNING  !!!              !!*/
/*!! Before building check :                  !!*/
/*!! libraries/VarioSettings/VarioSettings.h  !!*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/

/*uint8_t tmpValue=0;
unsigned long TmplastFreqUpdate;*/

/*******************/
/* General objects */
/*******************/
#define VARIOMETER_POWER_ON_DELAY 2000 

#define VARIOMETER_STATE_INITIAL 0
#define VARIOMETER_STATE_DATE_RECORDED 1
#define VARIOMETER_STATE_CALIBRATED 2
#define VARIOMETER_STATE_FLIGHT_STARTED 3

#ifdef HAVE_GPS
uint8_t variometerState = VARIOMETER_STATE_INITIAL;
#else
uint8_t variometerState = VARIOMETER_STATE_CALIBRATED;
#endif //HAVE_GPS

/*****************/
/* screen objets */
/*****************/
#ifdef HAVE_SCREEN

unsigned long lastLowFreqUpdate = 0;

#define RATIO_MAX_VALUE 30.0
#define RATIO_MIN_SPEED 10.0

#define VARIOSCREEN_ALTI_ANCHOR_X 110
#define VARIOSCREEN_ALTI_ANCHOR_Y 80
#define VARIOSCREEN_ALTI_UNIT_ANCHOR_X    120
#define VARIOSCREEN_VARIO_ANCHOR_X 90
#define VARIOSCREEN_VARIO_ANCHOR_Y 135
#define VARIOSCREEN_VARIO_UNIT_ANCHOR_X   95
#define VARIOSCREEN_VARIO_UNIT_ANCHOR_Y   110
#define VARIOSCREEN_SPEED_ANCHOR_X 50
#define VARIOSCREEN_SPEED_ANCHOR_Y 190
#define VARIOSCREEN_SPEED_UNIT_ANCHOR_X 55
#define VARIOSCREEN_SPEED_UNIT_ANCHOR_Y 165
#define VARIOSCREEN_GR_ANCHOR_X 144
#define VARIOSCREEN_GR_ANCHOR_Y 135
#define VARIOSCREEN_INFO_ANCHOR_X 4
#define VARIOSCREEN_INFO_ANCHOR_Y 0
#define VARIOSCREEN_VOL_ANCHOR_X 44
#define VARIOSCREEN_VOL_ANCHOR_Y 0
#define VARIOSCREEN_RECCORD_ANCHOR_X 84
#define VARIOSCREEN_RECCORD_ANCHOR_Y 0
#define VARIOSCREEN_BAT_ANCHOR_X 124
#define VARIOSCREEN_BAT_ANCHOR_Y 0
#define VARIOSCREEN_SAT_ANCHOR_X 164
#define VARIOSCREEN_SAT_ANCHOR_Y 0
#define VARIOSCREEN_SAT_FIX_ANCHOR_X 176
#define VARIOSCREEN_SAT_FIX_ANCHOR_Y 40
#define VARIOSCREEN_TIME_ANCHOR_X 195
#define VARIOSCREEN_TIME_ANCHOR_Y 190
#define VARIOSCREEN_ELAPSED_TIME_ANCHOR_X 197
#define VARIOSCREEN_ELAPSED_TIME_ANCHOR_Y 190
#define VARIOSCREEN_BT_ANCHOR_X 152
#define VARIOSCREEN_BT_ANCHOR_Y 40
#define VARIOSCREEN_TREND_ANCHOR_X 120
#define VARIOSCREEN_TREND_ANCHOR_Y 111

ScreenDigit altiDigit(screen, VARIOSCREEN_ALTI_ANCHOR_X, VARIOSCREEN_ALTI_ANCHOR_Y, 4, 0, false, ALIGNNONE);
MUnit munit(screen, VARIOSCREEN_ALTI_UNIT_ANCHOR_X, VARIOSCREEN_ALTI_ANCHOR_Y);
ScreenDigit varioDigit(screen, VARIOSCREEN_VARIO_ANCHOR_X, VARIOSCREEN_VARIO_ANCHOR_Y, 4, 1, true, ALIGNNONE);
MSUnit msunit(screen, VARIOSCREEN_VARIO_UNIT_ANCHOR_X, VARIOSCREEN_VARIO_UNIT_ANCHOR_Y);
KMHUnit kmhunit(screen, VARIOSCREEN_SPEED_UNIT_ANCHOR_X, VARIOSCREEN_SPEED_UNIT_ANCHOR_Y);
ScreenDigit speedDigit(screen, VARIOSCREEN_SPEED_ANCHOR_X, VARIOSCREEN_SPEED_ANCHOR_Y, 2, 0, false, ALIGNNONE);
ScreenDigit ratioDigit(screen, VARIOSCREEN_GR_ANCHOR_X, VARIOSCREEN_GR_ANCHOR_Y, 2, 0, false, ALIGNZERO,true);

INFOLevel infoLevel(screen, VARIOSCREEN_INFO_ANCHOR_X, VARIOSCREEN_INFO_ANCHOR_Y);
VOLLevel  volLevel(screen, VARIOSCREEN_VOL_ANCHOR_X, VARIOSCREEN_VOL_ANCHOR_Y);
RECORDIndicator recordIndicator(screen, VARIOSCREEN_RECCORD_ANCHOR_X, VARIOSCREEN_RECCORD_ANCHOR_Y);
TRENDLevel trendLevel(screen, VARIOSCREEN_TREND_ANCHOR_X, VARIOSCREEN_TREND_ANCHOR_Y);

BATLevel batLevel(screen, VARIOSCREEN_BAT_ANCHOR_X, VARIOSCREEN_BAT_ANCHOR_Y, VOLTAGE_DIVISOR_VALUE, VOLTAGE_DIVISOR_REF_VOLTAGE);
int maxVoltage = 0;
SATLevel satLevel(screen, VARIOSCREEN_SAT_ANCHOR_X, VARIOSCREEN_SAT_ANCHOR_Y);

ScreenDigit timeMDigit(screen, VARIOSCREEN_TIME_ANCHOR_X, VARIOSCREEN_TIME_ANCHOR_Y, 2, 0, false, ALIGNZERO);
ScreenDigit timeHDigit(screen, VARIOSCREEN_TIME_ANCHOR_X-68, VARIOSCREEN_TIME_ANCHOR_Y, 2, 0, false, ALIGNZERO);

ScreenTime screenTime(screen, VARIOSCREEN_TIME_ANCHOR_X, VARIOSCREEN_TIME_ANCHOR_Y, timeHDigit, timeMDigit);
ScreenElapsedTime screenElapsedTime(screen, VARIOSCREEN_ELAPSED_TIME_ANCHOR_X, VARIOSCREEN_ELAPSED_TIME_ANCHOR_Y, timeHDigit, timeMDigit);

FIXGPSInfo fixgpsinfo(screen, VARIOSCREEN_SAT_FIX_ANCHOR_X, VARIOSCREEN_SAT_FIX_ANCHOR_Y);
BTInfo btinfo(screen, VARIOSCREEN_BT_ANCHOR_X, VARIOSCREEN_BT_ANCHOR_Y);


ScreenSchedulerObject displayList[] = { {&msunit, 0,true}, {&munit, 0,true}, {&altiDigit, 0,true}, {&varioDigit, 0,true}
                       ,{&kmhunit, 0, true}, {&speedDigit, 0,true}, {&ratioDigit, 0,true}, {&satLevel, 0,true}, {&screenTime, 0,true}
                       ,{&screenElapsedTime, 0,false}, {&recordIndicator, 0,true} , {&trendLevel, 0,true}, {&batLevel, 0,true}
                       ,{&btinfo, 0,true}, {&fixgpsinfo, 0,true} , {&satLevel, 0,true}, {&volLevel, 0,true}, {&infoLevel,0,true}
};

ScreenScheduler varioScreen(screen, displayList, sizeof(displayList)/sizeof(ScreenSchedulerObject), 0, 0);

int8_t objectList[] = {8,9};
MultiDisplayObject multiDisplayList[] = {{objectList,sizeof(objectList)/sizeof(int8_t),0,0,2,2}};
MultiDisplay multiDisplay(displayList, sizeof(displayList)/sizeof(ScreenSchedulerObject), multiDisplayList, sizeof(multiDisplayList)/sizeof(MultiDisplayObject));


#endif //HAVE_SCREEN

/**********************/
/* alti/vario objects */
/**********************/
#define POSITION_MEASURE_STANDARD_DEVIATION 0.1
#ifdef HAVE_ACCELEROMETER 
#define ACCELERATION_MEASURE_STANDARD_DEVIATION 0.3

#define HIGH_BEEP_FREQ 1000.0
#define LOW_BEEP_FREQ 100.0
#define BASE_BEEP_DURATION 100.0

#define MEASURE_DELAY 3000 

AccelCalibrator calibrator;
#else
#define ACCELERATION_MEASURE_STANDARD_DEVIATION 0.6
#endif //HAVE_ACCELEROMETER 

kalmanvert kalmanvert;

#ifdef HAVE_SPEAKER
//beeper beeper(VARIOMETER_SINKING_THRESHOLD, VARIOMETER_CLIMBING_THRESHOLD, VARIOMETER_NEAR_CLIMBING_SENSITIVITY, VARIOMETER_BEEP_VOLUME);
Beeper beeper(10);

/*  uint8_t VARIOMETER_BEEP_VOLUME = 10;
  float VARIOMETER_SINKING_THRESHOLD =-2.0;
  float VARIOMETER_CLIMBING_THRESHOLD=0.2;
  float VARIOMETER_NEAR_CLIMBING_SENSITIVITY=0.5; */

#endif

/***************/
/* gps objects */
/***************/
#ifdef HAVE_GPS

NmeaParser nmeaParser;

#ifdef HAVE_BLUETOOTH
boolean lastSentence = false;
#endif //HAVE_BLUETOOTH

unsigned long RMCSentenceTimestamp; //for the speed filter
double RMCSentenceCurrentAlti; //for the speed filter
unsigned long speedFilterTimestamps[VARIOMETER_SPEED_FILTER_SIZE];
double speedFilterSpeedValues[VARIOMETER_SPEED_FILTER_SIZE];
double speedFilterAltiValues[VARIOMETER_SPEED_FILTER_SIZE];
int8_t speedFilterPos = 0;

#endif //HAVE_GPS

#ifdef HAVE_SDCARD
File file;

#ifdef HAVE_GPS
IGCHeader header;
IGCSentence igc;
#endif //HAVE_GPS

VarioSettings GnuSettings;

#define SDCARD_STATE_INITIAL 0
#define SDCARD_STATE_INITIALIZED 1
#define SDCARD_STATE_READY 2
#define SDCARD_STATE_ERROR -1
int8_t sdcardState = SDCARD_STATE_INITIAL;

#endif //HAVE_SDCARD


/*********************/
/* bluetooth objects */
/*********************/
#ifdef HAVE_BLUETOOTH
#if defined(VARIOMETER_SENT_LXNAV_SENTENCE)
LxnavSentence bluetoothNMEA;
#elif defined(VARIOMETER_SENT_LK8000_SENTENCE)
LK8Sentence bluetoothNMEA;
#else
#error No bluetooth sentence type specified !
#endif

#ifndef HAVE_GPS
unsigned long lastVarioSentenceTimestamp = 0;
#endif // !HAVE_GPS
#endif //HAVE_BLUETOOTH

/*-----------------*/
/*                 */
/*  displayboot    */
/*                 */
/*-----------------*/

void displayBoot(void) {
  char tmpbuffer[50];

#ifdef HAVE_SCREEN

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
#ifdef PROG_DEBUG
    Serial.print("update screen");
#endif //PRO_DEBBUG
  }

#endif //HAVE_SCREEN

}

/*-----------------*/
/*      SETUP      */
/*-----------------*/
void setup() {

/************************/
/*        Init Power    */
/************************/

//  statePower = HIGH;
  pinMode(VARIO_DETECT_USB, INPUT_PULLDOWN);

  pinMode(VARIO_PIN_ALIM, OUTPUT);
  digitalWrite(VARIO_PIN_ALIM, HIGH);   // turn on power cards )

  digitalWrite(VARIO_PIN_RST, HIGH);   // Hard Reset M0 )
  pinMode(VARIO_PIN_RST, OUTPUT);

  /*****************************/
  /* wait for devices power on */
  /*****************************/
  delay(VARIOMETER_POWER_ON_DELAY);

 #ifdef PROG_DEBUG
  char tmpbuffer[50];

  Serial.begin(9600);
  while (!Serial) { ;}
  sprintf(tmpbuffer,"SAMD21 MPU9250 MS5611 VARIO compiled on %s at %s", __DATE__, __TIME__);
  Serial.println(tmpbuffer);
  Serial.flush();
#endif //PRO_DEBBUG

  /****************/
  /* init speaker */
  /****************/
#ifdef PROG_DEBUG
  Serial.println("Initializing ToneAC");
#endif //PRO_DEBBUG
  
#ifdef HAVE_SPEAKER
  toneAC_init();
#endif

delay(1000);

  /**********************/
  /* init accelerometer */
  /**********************/
#ifdef PROG_DEBUG
  Serial.println("Initializing vertaccel");
#endif //PRO_DEBBUG
  
  Wire.begin();
#ifdef HAVE_ACCELEROMETER
  vertaccel_init();
#endif //HAVE_ACCELEROMETER

  /****************/
  /* init SD Card */
  /****************/
  
#ifdef HAVE_SDCARD
#ifdef PROG_DEBUG
  Serial.println("Initializing SD card...");
#endif //PRO_DEBBUG

  if (GnuSettings.initSettings()) {
#ifdef PROG_DEBUG
   Serial.println("initialization done.");
#endif //PROG_DEBUG

   GnuSettings.readSDSettings();
 
#ifdef PROG_DEBUG
   //Debuuging Printing
 Serial.print("Pilot Name = ");
 Serial.println(GnuSettings.VARIOMETER_PILOT_NAME);
#endif //PROG_DEBUG

    sdcardState = SDCARD_STATE_INITIALIZED;  //useless to set error
  }
  else
  {
#ifdef HAVE_SPEAKER
    if (GnuSettings.ALARM_SDCARD) {
#ifdef PROG_DEBUG
      Serial.println("initialization failed!");
#endif //PROG_DEBUG

      indicateFaultSDCARD();
    }
#endif //HAVE_SPEAKER 
  }  
#endif //HAVE_SDCARD


/******************/
/*    init Audio  */
/******************/
#ifdef HAVE_SPEAKER
beeper.init(GnuSettings.VARIOMETER_SINKING_THRESHOLD, GnuSettings.VARIOMETER_CLIMBING_THRESHOLD, GnuSettings.VARIOMETER_NEAR_CLIMBING_SENSITIVITY, GnuSettings.VARIOMETER_BEEP_VOLUME);

#ifdef PROG_DEBUG
  sprintf(tmpbuffer,"Volume : %i", GnuSettings.VARIOMETER_BEEP_VOLUME);
  Serial.println(tmpbuffer);
  Serial.flush();
#endif //PROG_DEBUG

beeper.setVolume(GnuSettings.VARIOMETER_BEEP_VOLUME);
#endif
 
  /***************/
  /* init screen */
  /***************/
#ifdef PROG_DEBUG
      Serial.println("initialization screen");
#endif //IMU_DEBUG

#ifdef HAVE_SCREEN
  screen.begin();

  varioScreen.setPage(0);

  recordIndicator.setActifSCAN();
  fixgpsinfo.unsetFixGps();
  infoLevel.set(INFO_NONE);

#ifdef HAVE_BLUETOOTH
     btinfo.setBT();
#endif //HAVE_BLUETOOTH

/*----------------------------------------*/
/*                                        */
/*             DISPLAY BOOT               */
/*                                        */
/*----------------------------------------*/

   displayBoot();  
#endif //HAVE_SCREEN

  /******************/
  /* Detect calibrated */
  /******************/

    // if we got this far, MPU9250 and MS5611 look OK
  // Read calibrated accelerometer and gyro bias values saved in M0 flash

   if (vertaccel_readAvailableCalibration() == false) {
    
#ifdef IMU_DEBUG
  Serial.println("Uncalibrated Accelerometre");
#endif //IMU_DEBUG

    indicateUncalibratedAccelerometer(); // series of alternating low/high tones
  }

  /*---------------------------------------------------------------------------------------------------*/
  /*                                                                                                   */
  /*                             CALIBRATION                                                           */
  /*                                                                                                   */
  /*---------------------------------------------------------------------------------------------------*/


#ifdef PROG_DEBUG
  int tmpvolume = beeper.getVolume();
  beeper.setVolume(10);
#endif



#ifdef HAVE_ACCELEROMETER
  if( MpuCalibrationCond() ) {
#ifdef HAVE_SPEAKER
#ifdef IMU_DEBUG
      Serial.println("Calibration");
#endif //IMU_DEBUG

    beeper.GenerateTone(GnuSettings.CALIB_TONE_FREQHZ, 3000);
    // allow 10 seconds for the unit to be placed in calibration position with the 
    // accelerometer +z pointing downwards. Indicate this delay with a series of short beeps
    for (int inx = 0; inx < 50; inx++) {
      delay(200); 
      beeper.GenerateTone(GnuSettings.CALIB_TONE_FREQHZ,50);
      }
#endif //HAVE_SPEAKER
#ifdef IMU_DEBUG
    Serial.println("Calibrating accel & gyro");
#endif IMU_DEBUG

  // Calibration des accÃ©lerometres

    calibrator.init();

  /* start beep */
    signalBeep(HIGH_BEEP_FREQ, BASE_BEEP_DURATION, 3);

  /***************************/
  /* make measure repeatedly */
  /***************************/
  
  /* wait for positionning accelerometer */
    delay(MEASURE_DELAY);
  
  /* make measure */
    calibrator.measure();
    
  /********************************************/
  /* the reversed position launch calibration */
  /********************************************/
  
  /* get orientation */
    int orient = calibrator.getMeasureOrientation();
    if( orient == ACCEL_CALIBRATOR_ORIENTATION_EXCEPTION ) {
    
    /**********************/
    /* launch calibration */
    /**********************/
      if( !calibrator.canCalibrate() ) {
      signalBeep(LOW_BEEP_FREQ, BASE_BEEP_DURATION, 3);
      } else {
        calibrator.calibrate();
        signalBeep(HIGH_BEEP_FREQ, BASE_BEEP_DURATION, 3);
      }
    
    } else {
    
    /****************/
    /* push measure */
    /****************/
    
      boolean measureValid = calibrator.pushMeasure();
          
      /* make corresponding beep */
      if( measureValid ) {
        signalBeep(HIGH_BEEP_FREQ, BASE_BEEP_DURATION*3, 1);
      } else {
        signalBeep(LOW_BEEP_FREQ, BASE_BEEP_DURATION*3, 1);
      }
    }
     
    /* start beep */
    signalBeep(HIGH_BEEP_FREQ, BASE_BEEP_DURATION, 3);

    GnuSettings.writeFlashSDSettings();

    // indicate calibration complete
#ifdef HAVE_SPEAKER
    beeper.GenerateTone(GnuSettings.CALIB_TONE_FREQHZ, 1000);
#endif //HAVE_SPEAKER

    NVIC_SystemReset();      // processor software reset 

  }
#endif //HAVE_ACCELEROMETER


#ifdef HAVE_SPEAKER
 beeper.setVolume(tmpvolume);
#endif //HAVE_SPEAKER
 
  /******************/
  /* init barometer */
  /******************/
#ifdef PROG_DEBUG
      Serial.println("initialization ms5611");
#endif //IMU_DEBUG
 ms5611_init();
  
  /**************************/
  /* init gps and bluetooth */
  /**************************/
#if defined(HAVE_BLUETOOTH) || defined(HAVE_GPS)
#ifdef PROG_DEBUG
      Serial.println("initialization gps");
#endif //IMU_DEBUG

#ifdef HAVE_GPS
  serialNmea.begin(GPS_BLUETOOTH_BAUDS, true);
#else
  serialNmea.begin(GPS_BLUETOOTH_BAUDS, false);
#endif //HAVE_GPS
#endif //defined(HAVE_BLUETOOTH) || defined(HAVE_GPS)
  
  /******************/
  /* get first data */
  /******************/

delay(1000);

  /* wait for first alti and acceleration */
#ifdef PROG_DEBUG
      Serial.println("first alti");
#endif //IMU_DEBUG
  
  while( ! (ms5611_dataReady()
#ifdef HAVE_ACCELEROMETER
            && vertaccel_dataReady()
#endif //HAVE_ACCELEROMETER
            ) ) {
  }
  
  /* get first data */
  ms5611_updateData();
#ifdef HAVE_ACCELEROMETER
  vertaccel_updateData();
#endif //HAVE_ACCELEROMETER

  /* init kalman filter */
#ifdef PROG_DEBUG
      Serial.println("initialization kalman");
#endif //IMU_DEBUG

  kalmanvert.init(ms5611_getAltitude(),
#ifdef HAVE_ACCELEROMETER
                  vertaccel_getValue(),
#else
                  0.0,
#endif
                  POSITION_MEASURE_STANDARD_DEVIATION,
                  ACCELERATION_MEASURE_STANDARD_DEVIATION,
                  millis());

#ifdef HAVE_SPEAKER
  beeper.setVolume(GnuSettings.VARIOMETER_BEEP_VOLUME);
#ifdef HAVE_SCREEN
  volLevel.setVolume(GnuSettings.VARIOMETER_BEEP_VOLUME);
#endif //HAVE_SCREEN
#else
  volLevel.setVolume(0);
#endif //HAVE_SPEAKER

toneAC(2000);
delay(1000);
noToneAC();  

/*----------------------------------------*/
/*                                        */
/*           CLEAR DISPLAY BOOT           */
/*                                        */
/*----------------------------------------*/
#ifdef IMU_DEBUG
    Serial.println("Clear Screen");
#endif IMU_DEBUG
  
//  delay(2000);
#ifdef HAVE_SCREEN
  screen.fillScreen(GxEPD_WHITE);
#endif //HAVE_SCREEN
}

#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
void createSDCardTrackFile(void);
#endif //defined(HAVE_SDCARD) && defined(HAVE_GPS)
void enableflightStartComponents(void);

/*----------------*/
/*      LOOP      */
/*----------------*/
void loop() {
 
  /*****************************/
  /* compute vertical velocity */
  /*****************************/
#ifdef HAVE_ACCELEROMETER
  if( ms5611_dataReady() && vertaccel_dataReady() ) {
    ms5611_updateData();
    vertaccel_updateData();

    kalmanvert.update( ms5611_getAltitude(),
                       vertaccel_getValue(),
                       millis() );
#else
  if( ms5611_dataReady() ) {
    ms5611_updateData();

    kalmanvert.update( ms5611_getAltitude(),
                       0.0,
                       millis() );
#endif //HAVE_ACCELEROMETER

    /* set beeper */
#ifdef HAVE_SPEAKER
    beeper.setVelocity( kalmanvert.getVelocity() );
#endif //HAVE_SPEAKER

    /* set screen */
#ifdef HAVE_SCREEN
    altiDigit.setValue(kalmanvert.getCalibratedPosition());
    varioDigit.setValue(kalmanvert.getVelocity() );

#endif //HAVE_SCREEN
   
    Serial.println(kalmanvert.getCalibratedPosition());
    Serial.println(kalmanvert.getVelocity() );
    
  }

  /*****************/
  /* update beeper */
  /*****************/
#ifdef HAVE_SPEAKER
  beeper.update();
#endif //HAVE_SPEAKER


  /********************/
  /* update bluetooth */
  /********************/
#ifdef HAVE_BLUETOOTH
#ifdef HAVE_GPS
  /* in priority send vario nmea sentence */
  if( bluetoothNMEA.available() ) {
    while( bluetoothNMEA.available() ) {
       serialNmea.write( bluetoothNMEA.get() );
    }
    serialNmea.release();
  }
#else //!HAVE_GPS
  /* check the last vario nmea sentence */
  if( millis() - lastVarioSentenceTimestamp > VARIOMETER_SENTENCE_DELAY ) {
    lastVarioSentenceTimestamp = millis();
#ifdef VARIOMETER_BLUETOOTH_SEND_CALIBRATED_ALTITUDE
    bluetoothNMEA.begin(kalmanvert.getCalibratedPosition(), kalmanvert.getVelocity());
#else
    bluetoothNMEA.begin(kalmanvert.getPosition(), kalmanvert.getVelocity());
#endif
    while( bluetoothNMEA.available() ) {
       serialNmea.write( bluetoothNMEA.get() );
    }
  }
#endif //!HAVE_GPS
#endif //HAVE_BLUETOOTH


  /**************/
  /* update GPS */
  /**************/
#ifdef HAVE_GPS
#ifdef HAVE_BLUETOOTH
  /* else try to parse GPS nmea */
  else {
#endif //HAVE_BLUETOOTH
    
    /* try to lock sentences */
    if( serialNmea.lockRMC() ) {
      RMCSentenceTimestamp = millis();
      RMCSentenceCurrentAlti = kalmanvert.getPosition(); //useless to take calibrated here
      nmeaParser.beginRMC();
    } else if( serialNmea.lockGGA() ) {
      nmeaParser.beginGGA();
#ifdef HAVE_BLUETOOTH
      lastSentence = true;
#endif //HAVE_BLUETOOTH
#ifdef HAVE_SDCARD      
      /* start to write IGC B frames */
      if( sdcardState == SDCARD_STATE_READY ) {
#ifdef VARIOMETER_SDCARD_SEND_CALIBRATED_ALTITUDE
        file.write( igc.begin( kalmanvert.getCalibratedPosition() ) );
#else
        file.write( igc.begin( kalmanvert.getPosition() ) );
#endif
      }
#endif //HAVE_SDCARD
    }
  
    /* parse if needed */
    if( nmeaParser.isParsing() ) {
      while( nmeaParser.isParsing() ) {
        uint8_t c = serialNmea.read();
        
        /* parse sentence */        
        nmeaParser.feed( c );
#ifdef PROG_DEBUG
      Serial.print(c);
#endif //IMU_DEBUG


#ifdef HAVE_SDCARD          
        /* if GGA, convert to IGC and write to sdcard */
        if( sdcardState == SDCARD_STATE_READY && nmeaParser.isParsingGGA() ) {
          igc.feed(c);
          while( igc.available() ) {           
            file.write( igc.get() );
          }
        }
#endif //HAVE_SDCARD
      }
#ifdef PROG_DEBUG
      Serial.println("");
#endif //IMU_DEBUG
      
      serialNmea.release();
    
#ifdef HAVE_BLUETOOTH   
      /* if this is the last GPS sentence */
      /* we can send our sentences */
      if( lastSentence ) {
          lastSentence = false;
#ifdef VARIOMETER_BLUETOOTH_SEND_CALIBRATED_ALTITUDE
          bluetoothNMEA.begin(kalmanvert.getCalibratedPosition(), kalmanvert.getVelocity());
#else
          bluetoothNMEA.begin(kalmanvert.getPosition(), kalmanvert.getVelocity());
#endif
          serialNmea.lock(); //will be writed at next loop
      }
#endif //HAVE_BLUETOOTH
    }
  
  
    /***************************/
    /* update variometer state */
    /*    (after parsing)      */
    /***************************/
    if( variometerState < VARIOMETER_STATE_FLIGHT_STARTED ) {

      /* if initial state check if date is recorded  */
      if( variometerState == VARIOMETER_STATE_INITIAL ) {
        if( nmeaParser.haveDate() ) {
          variometerState = VARIOMETER_STATE_DATE_RECORDED;
        }
      }
      
      /* check if we need to calibrate the altimeter */
      else if( variometerState == VARIOMETER_STATE_DATE_RECORDED ) {
        
        /* we need a good quality value */
        if( nmeaParser.haveNewAltiValue() && nmeaParser.precision < VARIOMETER_GPS_ALTI_CALIBRATION_PRECISION_THRESHOLD ) {
          
          /* calibrate */
 #ifdef HAVE_SPEAKER 
   if (GnuSettings.ALARM_GPSFIX) {
 //         toneAC(BEEP_FREQ);
          beeper.GenerateTone(GnuSettings.BEEP_FREQ, 200);
//          delay(200);
//          toneAC(0);
   }
 #endif //defined(HAVE_SPEAKER) 
 
#ifdef HAVE_SCREEN
          recordIndicator.setActifGPSFIX();
          fixgpsinfo.setFixGps();
#endif //HAVE_SCREEN                    
       
          double gpsAlti = nmeaParser.getAlti();
          kalmanvert.calibratePosition(gpsAlti);
          variometerState = VARIOMETER_STATE_CALIBRATED;
/*#if defined(HAVE_SDCARD) && ! defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)
          createSDCardTrackFile();
#endif //defined(HAVE_SDCARD) && ! defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)*/
#ifdef HAVE_SDCARD 
          if (!GnuSettings.VARIOMETER_RECORD_WHEN_FLIGHT_START) {
             createSDCardTrackFile();
          }
#endif //HAVE_SDCARD
        }
      }
      
      /* else check if the flight have started */
/*      else {  //variometerState == VARIOMETER_STATE_CALIBRATED
        
        /* check flight start condition *
        if( (millis() > GnuSettings.FLIGHT_START_MIN_TIMESTAMP) &&
            (kalmanvert.getVelocity() < GnuSettings.FLIGHT_START_VARIO_LOW_THRESHOLD || kalmanvert.getVelocity() > GnuSettings.FLIGHT_START_VARIO_HIGH_THRESHOLD) &&
            (nmeaParser.getSpeed() > GnuSettings.FLIGHT_START_MIN_SPEED) ) {
          variometerState = VARIOMETER_STATE_FLIGHT_STARTED;
          enableflightStartComponents();
        }
      }
    }
#ifdef HAVE_BLUETOOTH*/
      else {  //variometerState == VARIOMETER_STATE_CALIBRATED
        
        /* check flight start condition */
        if( (millis() > GnuSettings.FLIGHT_START_MIN_TIMESTAMP) &&
            ((GnuSettings.VARIOMETER_RECORD_WHEN_FLIGHT_START) &&   
             (kalmanvert.getVelocity() < GnuSettings.FLIGHT_START_VARIO_LOW_THRESHOLD || kalmanvert.getVelocity() > GnuSettings.FLIGHT_START_VARIO_HIGH_THRESHOLD)) ||
             (!GnuSettings.VARIOMETER_RECORD_WHEN_FLIGHT_START)
     
  //        && (kalmanvert.getVelocity() < FLIGHT_START_VARIO_LOW_THRESHOLD || kalmanvert.getVelocity() > FLIGHT_START_VARIO_HIGH_THRESHOLD) &&
#ifdef HAVE_GPS
            &&(nmeaParser.getSpeed() > GnuSettings.FLIGHT_START_MIN_SPEED) 
#endif //HAVE_GPS
          ) {
          variometerState = VARIOMETER_STATE_FLIGHT_STARTED;
          enableflightStartComponents();
        }
      }
    }
#ifdef HAVE_BLUETOOTH
  }
#endif //HAVE_BLUETOOTH
#endif //HAVE_GPS

  /* if no GPS, we can't calibrate, and we have juste to check flight start */
#ifndef HAVE_GPS
  if( variometerState == VARIOMETER_STATE_CALIBRATED ) { //already calibrated at start 
/*    if( (millis() > GnuSettings.FLIGHT_START_MIN_TIMESTAMP) &&
        (kalmanvert.getVelocity() < GnuSettings.FLIGHT_START_VARIO_LOW_THRESHOLD || kalmanvert.getVelocity() > GnuSettings.FLIGHT_START_VARIO_HIGH_THRESHOLD) ) {
      variometerState = VARIOMETER_STATE_FLIGHT_STARTED;
      enableflightStartComponents();*/
      if( (millis() > GnuSettings.FLIGHT_START_MIN_TIMESTAMP) &&
          (((GnuSettings.VARIOMETER_RECORD_WHEN_FLIGHT_START) &&   
           (kalmanvert.getVelocity() < GnuSettings.FLIGHT_START_VARIO_LOW_THRESHOLD || kalmanvert.getVelocity() > GnuSettings.FLIGHT_START_VARIO_HIGH_THRESHOLD)) || 
           (!GnuSettings.VARIOMETER_RECORD_WHEN_FLIGHT_START))) {
      variometerState = VARIOMETER_STATE_FLIGHT_STARTED;
      enableflightStartComponents();      
    }
  }
#endif // !HAVE_GPS

#if defined(HAVE_SCREEN) && defined(HAVE_VOLTAGE_DIVISOR) 
int tmpVoltage = analogRead(VOLTAGE_DIVISOR_PIN);
if (maxVoltage < tmpVoltage) {maxVoltage = tmpVoltage;}
#endif //HAVE_VOLTAGE_DIVISOR

  /**********************************/
  /* update low freq screen objects */
  /**********************************/
#ifdef HAVE_SCREEN

/************************************/
/* Update Time, duration            */
/* Voltage, SatLevel                */
/************************************/

#ifdef HAVE_GPS
      /* set time */
#ifdef PROG_DEBUG
      Serial.print("Time : ");
      Serial.println(nmeaParser.time);
#endif //IMU_DEBUG

      screenTime.setTime( nmeaParser.time );
      screenTime.correctTimeZone( GnuSettings.VARIOMETER_TIME_ZONE );
      screenElapsedTime.setCurrentTime( screenTime.getTime() );

      /* update satelite count */
      satLevel.setSatelliteCount( nmeaParser.satelliteCount );
#ifdef PROG_DEBUG
      Serial.print("Sat : ");
      Serial.println(nmeaParser.satelliteCount);
#endif //IMU_DEBUG
      
#endif //HAVE_GPS  

#ifdef HAVE_VOLTAGE_DIVISOR
      /* update battery level */
//      batLevel.setVoltage( analogRead(VOLTAGE_DIVISOR_PIN) );
      batLevel.setVoltage( maxVoltage );
      maxVoltage = 0;
#endif //HAVE_VOLTAGE_DIVISOR
   
  /*****************/
  /* update screen */
  /*****************/
#ifdef HAVE_GPS
  /* when getting speed from gps, display speed and ratio */
  if ( nmeaParser.haveNewSpeedValue() ) {

    /* get new values */
    unsigned long baseTime = speedFilterTimestamps[speedFilterPos];
    unsigned long deltaTime = RMCSentenceTimestamp; //delta computed later
    speedFilterTimestamps[speedFilterPos] = RMCSentenceTimestamp;
    
    double deltaAlti = speedFilterAltiValues[speedFilterPos]; //computed later
    speedFilterAltiValues[speedFilterPos] = RMCSentenceCurrentAlti; 

    double currentSpeed = nmeaParser.getSpeed();
    speedFilterSpeedValues[speedFilterPos] = currentSpeed;

    speedFilterPos++;
    if( speedFilterPos >= VARIOMETER_SPEED_FILTER_SIZE )
      speedFilterPos = 0;

    /* compute deltas */
    deltaAlti -= RMCSentenceCurrentAlti;
    deltaTime -= baseTime;
    
    /* compute mean distance */
    double meanDistance = 0;
    int step = 0;
    while( step < VARIOMETER_SPEED_FILTER_SIZE ) {

      /* compute distance */
      unsigned long currentTime = speedFilterTimestamps[speedFilterPos];
      meanDistance += speedFilterSpeedValues[speedFilterPos] * (double)(currentTime - baseTime);
      baseTime = currentTime;

      /* next */
      speedFilterPos++;
      if( speedFilterPos >= VARIOMETER_SPEED_FILTER_SIZE )
        speedFilterPos = 0;
      step++;
    }

    /* compute glide ratio */
    double ratio = (meanDistance/3600.0)/deltaAlti;

    /* display speed and ratio */    
#ifdef PROG_DEBUG
      Serial.print("Speed : ");
      Serial.println(currentSpeed);
#endif //IMU_DEBUG
    speedDigit.setValue( currentSpeed );
    if( currentSpeed >= RATIO_MIN_SPEED && ratio >= 0.0 && ratio < RATIO_MAX_VALUE ) {
      ratioDigit.setValue(ratio);
    } else {
      ratioDigit.setValue(0.0);
    }
  }
#endif //HAVE_GPS

    recordIndicator.stateRECORD();


/*  if (TmplastFreqUpdate == 0) TmplastFreqUpdate = millis();
  unsigned long TmpFreqDuration = millis() - TmplastFreqUpdate;
  if( TmpFreqDuration > 100 ) {
    TmplastFreqUpdate = millis();

      tmpValue++;
      if (tmpValue > 99) tmpValue = 0;
      ratioDigit.setValue(tmpValue);
#ifdef PROG_DEBUG
      Serial.print("tmpValue : ");
      Serial.println(tmpValue);
#endif //IMU_DEBUG
  }*/



  /* screen update */
     multiDisplay.displayStep();
     varioScreen.displayStep();
     screen.updateScreen();

#endif //HAVE_SCREEN 
}



#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
void createSDCardTrackFile(void) {
  /* start the sdcard record */
  if( sdcardState == SDCARD_STATE_INITIALIZED ) {

    /* build date : convert from DDMMYY to YYMMDD */
    uint8_t dateChar[8]; //two bytes are used for incrementing number on filename
    uint8_t* dateCharP = dateChar;
    uint32_t date = nmeaParser.date;
    for(uint8_t i=0; i<3; i++) {
      uint8_t num = ((uint8_t)(date%100));
      dateCharP[0] = (num/10) + '0';
      dateCharP[1] = (num%10) + '0';
      dateCharP += 2;
      date /= 100;
    }

    /* create file */    
    file = SD.open((char*)dateChar, FILE_WRITE);
    if (file) {
      sdcardState = SDCARD_STATE_READY;
            
      /* write the header */
      int16_t datePos = header.begin();
      if( datePos >= 0 ) {
        while( datePos ) {
        file.write(header.get());
          datePos--;
        }

        /* write date : DDMMYY */
        uint8_t* dateCharP = &dateChar[4];
        for(int i=0; i<3; i++) {
          file.write(dateCharP[0]);
          file.write(dateCharP[1]);
          header.get();
          header.get();
          dateCharP -= 2;
        }
            
        while( header.available() ) {
          file.write(header.get());
        }
      }
    } else {
      sdcardState = SDCARD_STATE_ERROR; //avoid retry 
    }
  }
}
#endif //defined(HAVE_SDCARD) && defined(HAVE_GPS)



void enableflightStartComponents(void) {
  
#ifdef HAVE_SPEAKER
if (GnuSettings.ALARM_FLYBEGIN) {
  for( int i = 0; i<2; i++) {
  //   toneAC(BEEP_FREQ);
 //    delay(200);
  //   toneAC(0);
     beeper.GenerateTone(GnuSettings.BEEP_FREQ, 200);
     delay(200);
  }
}
#endif //HAVE_SPEAKER 
  
//display record
#ifdef HAVE_SCREEN
recordIndicator.setActifRECORD();
fixgpsinfo.setFixGps();
#endif //HAVE_SCREEN

  /* set base time */
#if defined(HAVE_SCREEN) && defined(HAVE_GPS)
  screenElapsedTime.setBaseTime( screenTime.getTime() );
#endif //defined(HAVE_SCREEN) && defined(HAVE_GPS)

  /* enable near climbing */
#ifdef HAVE_SPEAKER
//#ifdef VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM
if (GnuSettings.VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM) {
  beeper.setGlidingAlarmState(true);
}
//#endif

//#ifdef VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP
if (GnuSettings.VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP) {
  beeper.setGlidingBeepState(true);
}
//#endif
#endif //HAVE_SPEAKER

#if defined(HAVE_SDCARD) && defined(HAVE_GPS) 
//&& defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)
if (GnuSettings.VARIOMETER_RECORD_WHEN_FLIGHT_START) {
  createSDCardTrackFile();
}
#endif // defined(HAVE_SDCARD) && defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)
}

