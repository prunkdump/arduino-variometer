// CJMCU-117 MPU9250+MS5611 circuit interface
//
// VCC  VCC
// GND  GND
// SCL  D12 - SCL
// SDA  D11 - SDA
//
// LM9110
// PWM   A3, A4 PWM
//
// A1,A2 Switch
// D1    Detection ON/OFF
//
// A6    Detection de connection USB
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

#include <SDU.h> 

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <reset.h>
#include <SD.h>

#include <FlashStorage.h>
#include <VarioSettings.h>

#include <SerialNmea_zero.h>   

#include <I2Cdev.h>
#include <ms5611_zero.h>
#include <vertaccel.h>
#include <kalmanvert.h>

#include <toneACZero.h>
#include <beeper.h>

#include <NmeaParser.h>
#include <LxnavSentence.h>
#include <IGCSentence.h>
#include <LK8Sentence.h>
#include <VarioComm.h>

#include <accelcalibrator.h>

#include <RTCZero.h>

/*****************/
/* screen        */
/*****************/
#ifdef HAVE_SCREEN

// include library, include base class, make path known
#include <GxEPD.h>

// select the display class to use, only one
#include <GxGDEP015OC1/GxGDEP015OC1.cpp>
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
/* v 63.0     beta version
 * 
 *******************/

 #ifdef IMU_DEBUG
 double timebegin;
 #endif

/*******************/
/* General objects */
/*******************/
#define VARIOMETER_STATE_INITIAL 0
#define VARIOMETER_STATE_DATE_RECORDED 1
#define VARIOMETER_STATE_CALIBRATED 2
#define VARIOMETER_STATE_FLIGHT_STARTED 3

#ifdef HAVE_GPS
uint8_t variometerState = VARIOMETER_STATE_INITIAL;
#else
uint8_t variometerState = VARIOMETER_STATE_CALIBRATED;
#endif //HAVE_GPS

VarioComm variocomm;

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
#define VARIOSCREEN_GR_ANCHOR_X 195
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
#define VARIOSCREEN_TIME_ANCHOR_X 197
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
ScreenDigit ratioDigit(screen, VARIOSCREEN_GR_ANCHOR_X, VARIOSCREEN_GR_ANCHOR_Y, 2, 0, false, ALIGNNONE);

INFOLevel infoLevel(screen, VARIOSCREEN_INFO_ANCHOR_X, VARIOSCREEN_INFO_ANCHOR_Y);
VOLLevel  volLevel(screen, VARIOSCREEN_VOL_ANCHOR_X, VARIOSCREEN_VOL_ANCHOR_Y);
RECORDIndicator recordIndicator(screen, VARIOSCREEN_RECCORD_ANCHOR_X, VARIOSCREEN_RECCORD_ANCHOR_Y);
TRENDLevel trendLevel(screen, VARIOSCREEN_TREND_ANCHOR_X, VARIOSCREEN_TREND_ANCHOR_Y);

BATLevel batLevel(screen, VARIOSCREEN_BAT_ANCHOR_X, VARIOSCREEN_BAT_ANCHOR_Y, VOLTAGE_DIVISOR_VALUE, VOLTAGE_DIVISOR_REF_VOLTAGE);
int maxVoltage = 0;
SATLevel satLevel(screen, VARIOSCREEN_SAT_ANCHOR_X, VARIOSCREEN_SAT_ANCHOR_Y);

ScreenDigit timeMDigit(screen, VARIOSCREEN_TIME_ANCHOR_X, VARIOSCREEN_TIME_ANCHOR_Y, 2, 0, false, ALIGNZERO);
ScreenDigit timeHDigit(screen, VARIOSCREEN_TIME_ANCHOR_X-64, VARIOSCREEN_TIME_ANCHOR_Y, 2, 0, false, ALIGNZERO);

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
AccelCalibrator calibrator;
#else
#define ACCELERATION_MEASURE_STANDARD_DEVIATION 0.6
#endif //HAVE_ACCELEROMETER 

kalmanvert kalmanvert;

uint32_t timePreviousUs;
uint32_t timeNowUs;
float imuTimeDeltaSecs;

void initTime() {
	timeNowUs = timePreviousUs = micros();
	}

void updateTime(){
	timeNowUs = micros();
	imuTimeDeltaSecs = ((timeNowUs - timePreviousUs) / 1000000.0f);
	timePreviousUs = timeNowUs;
	}

#ifdef HAVE_SDCARD
File file;
#endif //HAVE_SDCARD

/*-----------------------------------*/
/* Initialisation Button             */
/*-----------------------------------*/

byte statePowerInt = LOW;
byte statePower;

uint32_t timeNowPower;

void POWERInterruptHandler() {

  if (statePowerInt == LOW) {
    if (statePower == HIGH) {   
      statePowerInt = HIGH;
      timeNowPower = millis(); 
    }
    else {   
      pinMode(LED_BUILTIN, OUTPUT);
 
      digitalWrite (LED_BUILTIN, HIGH);

         
//      initiateReset(1);
//      tickReset();
    }
  } 
} 

byte stateLeftInterrup = LOW;

void LEFTInterruptHandler() {
  stateLeftInterrup = HIGH;
}

byte stateRightInterrup = LOW;

void RIGHTInterruptHandler() {
  stateRightInterrup = HIGH;
}
/**********************/
/* sound objects */
/**********************/

#ifdef HAVE_SPEAKER
//beeper beeper(VARIOMETER_SINKING_THRESHOLD, VARIOMETER_CLIMBING_THRESHOLD, VARIOMETER_NEAR_CLIMBING_SENSITIVITY, VARIOMETER_BEEP_VOLUME);
beeper beeper;
#endif

// if imu calibration data in flash is corrupted, the accel and gyro biases are 
// set to 0, and this uncalibrated state is indicated with a sequence of alternating 
// low and high beeps.
void indicateUncalibratedAccelerometer() {
  for (int cnt = 0; cnt < 5; cnt++) {
#ifdef HAVE_SPEAKER
    beeper.GenerateTone(200,100); 
    beeper.GenerateTone(2000,100);
#endif //HAVE_SPEAKER
    }
  }
  
// "no-activity" power down is indicated with a series of descending
// tones. If you hear this, switch off the vario as there is still
// residual current draw from the circuit components  
void indicatePowerDown() {
#ifdef HAVE_SPEAKER
  beeper.GenerateTone(2000,1000); 
  beeper.GenerateTone(1000,1000);
  beeper.GenerateTone(500, 1000);
  beeper.GenerateTone(250, 1000);
#endif //HAVE_SPEAKER
  }

// problem with MS5611 calibration CRC, assume communication 
// error or bad device. Indicate with series of 10 high pitched beeps.
void indicateFaultMS5611() {
#ifdef HAVE_SPEAKER
  for (int cnt = 0; cnt < 10; cnt++) {
    beeper.GenerateTone(GnuSettings.MS5611_ERROR_TONE_FREQHZ,1000); 
    delay(100);
    }
#endif //HAVE_SPEAKER    
  }

// problem reading MPU9250 ID, assume communication 
// error or bad device. Indicate with series of 10 low pitched beeps.
void indicateFaultMPU9250() {
#ifdef HAVE_SPEAKER
  for (int cnt = 0; cnt < 10; cnt++) {
    beeper.GenerateTone(GnuSettings.MPU9250_ERROR_TONE_FREQHZ,1000); 
    delay(100);
    }
#endif //HAVE_SPEAKER    
  }


// problem SDCARD, assume communication 
// error or bad device. Indicate with series of 10 low pitched beeps.
void indicateFaultSDCARD() {
#ifdef HAVE_SPEAKER
  for (int cnt = 0; cnt < 10; cnt++) {
    beeper.GenerateTone(GnuSettings.SDCARD_ERROR_TONE_FREQHZ,1000); 
    delay(100);
    }
#endif //HAVE_SPEAKER
  }

/* make beeps */
void signalBeep(double freq, unsigned long duration, int count = 1) {
 
  toneAC(freq);
  delay(duration);
  toneAC(0.0);
  if( count > 1 ) {
    while( count > 1 ) {
      delay(duration);
      toneAC(freq);
      delay(duration);
      toneAC(0.0);
      count--;
    }
  }
}

RTCZero rtc;

void powerDown() {
  // Mettre le SAMD21 en mode veille

 #ifdef PROG_DEBUG
   Serial.print("Mise en veille");
#endif //PRO_DEBBUG

   digitalWrite(VARIO_PIN_ALIM, LOW);   // turn on power cards )

   detachInterrupt(digitalPinToInterrupt(VARIOBTN_LEFT_PIN));
   detachInterrupt(digitalPinToInterrupt(VARIOBTN_RIGHT_PIN));

   statePower = LOW;

   rtc.standbyMode();

#ifdef IMU_DEBUG
    Serial.println("Sortie du mode veille");
#endif IMU_DEBUG

 }

/**********************/
/* sensor objects */
/**********************/

/*Battery voltage * 10 */
int batteryVoltage(void) {
  int adcSample = 0;
  for (int inx = 0; inx < 4; inx++) {
    adcSample += analogRead(VOLTAGE_DIVISOR_PIN);
    delay(1);
    }
  adcSample /= 4;
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
  return (int) ((adcSample*4.3f*10.0f)/1023.0f + 0.5f); //  voltage x 10
  }  

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

#ifdef HAVE_SDCARD
VarioSettings GnuSettings;

IGCHeader header;
IGCSentence igc;

#define SDCARD_STATE_INITIAL 0
#define SDCARD_STATE_INITIALIZED 1
#define SDCARD_STATE_READY 2
#define SDCARD_STATE_ERROR -1
int8_t sdcardState = SDCARD_STATE_INITIAL;

#endif //HAVE_SDCARD

#endif //HAVE_GPS

/*********************/
/* bluetooth objects */
/*********************/
#ifdef HAVE_BLUETOOTH
#if defined(VARIOMETER_SENT_LXNAV_SENTENCE)
LxnavSentence bluetoothNMEA;
#elif defined(VARIOMETER_SENT_LK8000_SENTENCE)
LK8Sentence bluetoothNMEA;
#else
//#error No bluetooth sentence type specified !
#endif
#endif //HAVE_BLUETOOTH

#ifndef HAVE_GPS
unsigned long lastVarioSentenceTimestamp = 0;
#endif // !HAVE_GPS

unsigned long TmplastFreqUpdate;

Statistic GnuStatistic;

/*-----------------*/
/*                 */
/*      SETUP      */
/*                 */
/*-----------------*/
  
void setup() {
  
#ifdef IMU_DEBUG
  timebegin = millis();

	Serial.begin(9600);
  while (!Serial) { ;}
  char tmpbuffer[50];
	sprintf(tmpbuffer,"SAMD21 MPU9250 MS5611 VARIO compiled on %s at %s", __DATE__, __TIME__);
  Serial.println(tmpbuffer);
#endif //IMU_DEBUG

/************************/
/*        Init Power    */
/************************/

  statePower = HIGH;
  pinMode(VARIO_DETECT_USB, INPUT_PULLDOWN);

  pinMode(VARIO_PIN_ALIM, OUTPUT);
  digitalWrite(VARIO_PIN_ALIM, HIGH);   // turn on power cards )


/*************************/
/*  Init interruption    */
/*     POWER ON/OFF      */
/*************************/

  pinMode(VARIOPOWER_INT_PIN, INPUT_PULLDOWN);
  statePowerInt = LOW;
  attachInterrupt(digitalPinToInterrupt(VARIOPOWER_INT_PIN), POWERInterruptHandler, RISING);

/*************************/
/*  Init interruption    */
/*     BUTTON LEFT       */
/*************************/

  pinMode(VARIOBTN_LEFT_PIN, INPUT_PULLDOWN);
  stateLeftInterrup = LOW;
  attachInterrupt(digitalPinToInterrupt(VARIOBTN_LEFT_PIN), LEFTInterruptHandler, RISING);

/*************************/
/*  Init interruption    */
/*     BUTTON LEFT       */
/*************************/

  pinMode(VARIOBTN_RIGHT_PIN, INPUT_PULLDOWN);
  stateRightInterrup = LOW;
  attachInterrupt(digitalPinToInterrupt(VARIOBTN_RIGHT_PIN), RIGHTInterruptHandler, RISING);

  /*********************/
  /* init Standby Mode */
  /*********************/
  
   // Configure the regulator to run in normal mode when in standby mode
  // Otherwise it defaults to low power mode and can only supply 50 uA
  SYSCTRL->VREG.bit.RUNSTDBY = 1;

  // Enable the DFLL48M clock in standby mode
  SYSCTRL->DFLLCTRL.bit.RUNSTDBY = 1;

  // Disable the USB device, this avoids USB interrupts
  // mainly the SOF every 1ms.
  // Note: you'll have to double tap the reset button to load new sketches
//  USBDevice.detach();

  rtc.begin();
 
  
  /****************/
  /* init SD Card */
  /****************/
  
#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
#ifdef PROG_DEBUG
  Serial.print("Initializing SD card...");
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
#endif //defined(HAVE_SDCARD) && defined(HAVE_GPS)

 /******************/
/*    init Audio  */
/******************/
#ifdef HAVE_SPEAKER
beeper.init(GnuSettings.VARIOMETER_SINKING_THRESHOLD, GnuSettings.VARIOMETER_CLIMBING_THRESHOLD, GnuSettings.VARIOMETER_NEAR_CLIMBING_SENSITIVITY, GnuSettings.VARIOMETER_BEEP_VOLUME);
/*  toneAC_initClock();
  toneAC_init();*/
#endif

  /***************/
  /* init screen */
  /***************/
#ifdef HAVE_SCREEN
  screen.begin();

  int8_t tmptime[] = {0,SUB_VERSION,VERSION};
  screenTime.setTime(tmptime);

  varioScreen.setPage(0);

  recordIndicator.setActifSCAN();
  fixgpsinfo.unsetFixGps();
  infoLevel.set(INFO_NONE);

#ifdef HAVE_BLUETOOTH
     btinfo.setBT();
#endif //HAVE_BLUETOOTH

#ifdef HAVE_SPEAKER
  beeper.setVolume(8);
#else
  volLevel.setVolume(0);
#endif //HAVE_SPEAKER


/*----------------------------------------*/
/*                                        */
/*             DISPLAY BOOT               */
/*                                        */
/*----------------------------------------*/

  screen.drawBitmap(100, 10, logo_gnuvario, 102, 74, GxEPD_WHITE); //94

  screen.setFont(&FreeSansBold12pt7b);

  screen.setCursor(100, 30);
  screen.println("Version");
  screen.setCursor(120, 50);
  screen.println(" Beta");
  sprintf(tmpbuffer,"%02d.%02d", VERSION, SUB_VERSION);
  screen.setCursor(125, 70);
  screen.println(tmpbuffer);
  sprintf(tmpbuffer,"%s", __DATE__);
  screen.setCursor(25, 110);
  screen.println(tmpbuffer);

//  screen.update();
  screen.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);


#endif //HAVE_SCREEN


  /************************************/
  /* init altimeter and accelerometer */
  /************************************/

  Wire.begin();
  Wire.setClock(400000); // set clock frequency AFTER Wire.begin()
  ms5611_init();


#ifdef IMU_DEBUG
  Serial.println("Init MS5611");
#endif //IMU_DEBUG
  
#ifdef HAVE_ACCELEROMETER
 // vertaccel_init();

#ifdef IMU_DEBUG
  Serial.println("Init MPU9250");
#endif //IMU_DEBUG

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
  /*                             TEST CALIBRATION                                                      */
  /*                                                                                                   */
  /*---------------------------------------------------------------------------------------------------*/

  
  int bCalibrateAccelerometer = 0;
  // short beeps for ~5 seconds
  for (int inx = 0; inx < 10; inx++) {
    delay(500); 
#ifdef HAVE_SPEAKER
    beeper.GenerateTone(GnuSettings.CALIB_TONE_FREQHZ,50); 
#endif //HAVE_SPEAKER
    if (stateRightInterrup == HIGH) {
      
#ifdef IMU_DEBUG
      Serial.println("Right Button detected");
#endif //IMU_DEBUG

      delay(100); // debounce the button
      if (digitalRead (VARIOBTN_RIGHT_PIN) == HIGH) {
        
#ifdef IMU_DEBUG
        Serial.println("Second Right Button detected");
#endif //IMU_DEBUG

        bCalibrateAccelerometer = 1;
        stateRightInterrup = LOW;
        break;
      }
      stateRightInterrup = LOW;    
    }
  }
    
  if (bCalibrateAccelerometer) {  
    // acknowledge calibration button press with long tone
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

  
    // indicate calibration complete
#ifdef HAVE_SPEAKER
    beeper.GenerateTone(GnuSettings.CALIB_TONE_FREQHZ, 1000);
#endif //HAVE_SPEAKER
  }
#endif //HAVE_ACCELEROMETER
 
  /**************************/
  /* init gps and bluetooth */
  /**************************/
#if defined(HAVE_BLUETOOTH) || defined(HAVE_GPS)

  Serial1.begin(GPS_BLUETOOTH_BAUDS);

#ifdef HAVE_GPS
  serialNmea.begin(GPS_BLUETOOTH_BAUDS, true);
#else
  serialNmea.begin(GPS_BLUETOOTH_BAUDS, false);
#endif //HAVE_GPS
#endif //defined(HAVE_BLUETOOTH) || defined(HAVE_GPS)
  
  /******************/
  /* get first data */
  /******************/
  
  /* wait for first alti and acceleration */
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
  volLevel.setVolume(GnuSettings.VARIOMETER_BEEP_VOLUME);
#else
  volLevel.setVolume(0);
#endif //HAVE_SPEAKER


/*----------------------------------------*/
/*                                        */
/*           CLEAR DISPLAY BOOT           */
/*                                        */
/*----------------------------------------*/
  
//  delay(2000);
  screen.fillScreen(GxEPD_WHITE);
}

#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
void createSDCardTrackFile(void);
#endif //defined(HAVE_SDCARD) && defined(HAVE_GPS)
void enableflightStartComponents(void);
void usbConnectedMode(void);
boolean statistiquePage(void);
void soundPage(void);
void varioCode(void);
void displaySound(int8_t volume);


/*-----------------*/
/*                 */
/*      LOOP       */
/*                 */
/*-----------------*/


void loop(){

/*----------------------------------------*/
/*                                        */
/*      DETECTION LIAISON SERIE           */
/*         AVEC LOGICIEL PC               */
/*----------------------------------------*/
  if (digitalRead(VARIO_DETECT_USB) == HIGH) {

    /* Usb connected */

    usbConnectedMode();
  }


/*************************/
/*   Detect push button  */
/*                       */
/*************************/

  // check if the pushbutton is pressed.
  if (stateLeftInterrup == HIGH) {
     stateLeftInterrup = LOW;
    //Action - Left button press

    if (varioScreen.getPage() == 0) {

      if (statistiquePage()) {
        if (variometerState == VARIOMETER_STATE_FLIGHT_STARTED) {

#ifdef HAVE_SDCARD
          //Enregistrement CRC IGC
#endif //HAVE_SDCARD
        }

#ifdef HAVE_SDCARD
          //Fermeture fichier
          file.close();
#endif //HAVE_SDCARD
        
	      //Sleeping M0
        //Power OFF
        statePowerInt = LOW;
    
        //Mise en veille
        indicatePowerDown();
        powerDown();
	    }
    }
    else
    {
      varioScreen.previousPage();
    }
  }
  else if (stateRightInterrup == HIGH) {
  // check if the pushbutton is pressed.
    stateRightInterrup = LOW;
    //Action - Right button press

    if (varioScreen.getPage() == varioScreen.getMaxPage()) {
      soundPage();
    }
    else
    {
      varioScreen.nextPage();
    }
    
  } 
  else if (statePowerInt == HIGH) {
    if (statePower == LOW) {  
      //Power ON
      statePowerInt = LOW;
      initiateReset(1);
      tickReset();  
    }  
  }

  varioCode();
 
#ifdef HAVE_SCREEN

  /* screen update */
     multiDisplay.displayStep();
     varioScreen.displayStep();
     screen.updateScreen();

 /* if (TmplastFreqUpdate == 0) TmplastFreqUpdate = millis();
  unsigned long TmpFreqDuration = millis() - TmplastFreqUpdate;
  if( TmpFreqDuration > 50 ) {
    TmplastFreqUpdate = millis();

    drdyFlag = 1; // indicate new MPU9250 data is available
    drdyCounter++;
  }*/

#endif //HAVE_SCREEN 
}


/*----------------------*/
/*                      */
/*      VARIOCODE       */
/*                      */
/*----------------------*/


void varioCode(void) {
    /*****************************/
  /* compute vertical velocity */
  /*****************************/

    /*******************/
    /*Acquisition Data */
    /*******************/
#ifdef IMU_DEBUG
    Serial.println("ACQUISITION");
#endif //IMU_DEBUG

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

  GnuStatistic.setAlti(kalmanvert.getCalibratedPosition());
  GnuStatistic.setVario(kalmanvert.getVelocity());
    /* set screen */
#ifdef HAVE_SCREEN
      altiDigit.setValue(kalmanvert.getCalibratedPosition());
      varioDigit.setValue(kalmanvert.getVelocity());  //kfClimbrateCps/10);
#ifdef IMU_DEBUG
      Serial.print("altitude : ");
      Serial.print(kalmanvert.getCalibratedPosition());
      Serial.print("    -   Vario : ");
      Serial.println(kalmanvert.getVelocity());
#endif
#endif //HAVE_SCREEN
    
    
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
#ifdef HAVE_SDCARD 
  if (!GnuSettings.VARIOMETER_RECORD_WHEN_FLIGHT_START) {
          createSDCardTrackFile();
  }
#endif //HAVE_SDCARD
        }
      }
      
      /* else check if the flight have started */
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
 /*   if( (millis() > FLIGHT_START_MIN_TIMESTAMP) 
#if defined ( VARIOMETER_RECORD_WHEN_FLIGHT_START )      
      && (kalmanvert.getVelocity() < FLIGHT_START_VARIO_LOW_THRESHOLD || kalmanvert.getVelocity() > FLIGHT_START_VARIO_HIGH_THRESHOLD) 
#endif   //defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)           */
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
      screenTime.setTime( nmeaParser.time );
      screenTime.correctTimeZone( GnuSettings.VARIOMETER_TIME_ZONE );
      screenElapsedTime.setCurrentTime( screenTime.getTime() );

      /* update satelite count */
      satLevel.setSatelliteCount( nmeaParser.satelliteCount );
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
    GnuStatistic.setSpeed(currentSpeed);

    double meanAltitude = 0;
    double currentAlti  = RMCSentenceCurrentAlti;

    /*compute trend */
    
    int8_t previousAltiPos = ((int8_t)speedFilterPos) - 3;
    if( previousAltiPos < 0 ) {
      previousAltiPos += VARIOMETER_SPEED_FILTER_SIZE;
    }
    double previousAlti = speedFilterAltiValues[previousAltiPos];   
    int8_t trend;

    if ( ( (currentAlti - previousAlti) >= -1) && ( (currentAlti - previousAlti) <= 3)) {trend = 0;}
    else if ((currentAlti - previousAlti) < -1) { trend = -1;}
    else { trend = 1;}
    
    /* display trend */
    trendLevel.stateTREND( trend );

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
    speedDigit.setValue( currentSpeed );
    if( currentSpeed >= RATIO_MIN_SPEED && ratio >= 0.0 && ratio < RATIO_MAX_VALUE ) {
      ratioDigit.setValue(ratio);
    } else {
      ratioDigit.setValue(0.0);
    }
  }


#endif //HAVE_GPS

 recordIndicator.stateRECORD();

#endif //HAVE_SCREEN 

}

/*--------------------------------------*/
/*                                      */
/*      createSDCardTrackFile           */
/*                                      */
/*--------------------------------------*/


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


/*--------------------------------------*/
/*                                      */
/*      enableflightStartComponents     */
/*                                      */
/*--------------------------------------*/


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
  GnuStatistic.setTime(screenTime.getTime());
#endif //defined(HAVE_SCREEN) && defined(HAVE_GPS)

  /* enable near climbing */
if (GnuSettings.VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM) {
  beeper.setGlidingAlarmState(true);
}

if (GnuSettings.VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP) {
  beeper.setGlidingBeepState(true);
}

#if defined(HAVE_SDCARD) && defined(HAVE_GPS) 
if (GnuSettings.VARIOMETER_RECORD_WHEN_FLIGHT_START) {
  createSDCardTrackFile();
}
#endif // defined(HAVE_SDCARD) 
}


/*--------------------------------------*/
/*                                      */
/*            UsbConnectedMode          */
/*                                      */
/*--------------------------------------*/

void usbConnectedMode(void) {
  
  while (1) {
    
      /* detection passage en mode transfers PC <-> M0 */
/*    variocomm.setup();
    
   if (stateLeftInterrup == HIGH) {
    stateLeftInterrup = LOW;
    //Action - Left button press
  } 

  // check if the pushbutton is pressed.
  if (stateRightInterrup == HIGH) {
    stateRightInterrup = LOW;
    //Action - Right button press
  }   
      
*/
    break;  
  }   
}


/*--------------------------------------*/
/*                                      */
/*            statistiquePage           */
/*                                      */
/*--------------------------------------*/

boolean statistiquePage(void) {

char tmpbuffer[50];

#ifdef SPEAKER
  toneAC_notone();
#endif //SPEAKER

#ifdef IMU_DEBUG
    Serial.println("Statistique");
#endif IMU_DEBUG
   
#ifdef HAVE_SCREEN

  screen.fillRect(0, 0, GxEPD_WIDTH-1, GxEPD_HEIGHT-1, GxEPD_WHITE);
  screen.drawBitmap(36, 50, power, 128, 128, GxEPD_WHITE); 
  screen.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);

#endif //HAVE_SCREEN

  while (true) {

#ifdef IMU_DEBUG
       Serial.println("boucle ");
#endif IMU_DEBUG

    // check if the pushbutton is pressed.
    if (stateRightInterrup == HIGH) {
      delay(200);
      stateRightInterrup = LOW;
      //Action - Right button press
      screen.fillScreen(GxEPD_WHITE); 
	    varioScreen.setPage(0,true);
	    break;     
    } 
    else if (statePowerInt == HIGH) {
      delay(200);
      statePowerInt == LOW;

      screen.fillRect(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, GxEPD_WHITE);
      screen.drawBitmap(140, 10, logo_gnuvario50, 50, 36, GxEPD_WHITE); //94
      screen.setFont(&FreeSansBold12pt7b);
      screen.setTextSize(1);

      screen.setCursor(70, 30);
      int8_t timeValue[3];
      GnuStatistic.getTime(timeValue);
      sprintf(tmpbuffer,"H: %02d : %02d", timeValue[0], timeValue[1]);
      screen.println(tmpbuffer);
      screen.setCursor(70, 50);
      GnuStatistic.getDuration(timeValue);
      sprintf(tmpbuffer,"D: %02d : %02d", timeValue[0], timeValue[1]);
      screen.println(tmpbuffer);    
      double TmpValue    = GnuStatistic.getAltiDeco();
      double TmpValueMax = GnuStatistic.getMaxAlti();
      double TmpValueMin = GnuStatistic.getMinAlti();  
      sprintf(tmpbuffer,"Deco : %4.0f m", TmpValue );
      screen.setCursor(5, 75);
      screen.println(tmpbuffer);
      sprintf(tmpbuffer,"Alti m: %4.0f / %4.0f", TmpValueMin, TmpValueMax );
      screen.setCursor(5, 95);
      screen.println(tmpbuffer);
      TmpValue    = GnuStatistic.getGain();
      sprintf(tmpbuffer,"Gain : %4.0f m", TmpValue);
      screen.setCursor(5, 120);
      screen.println(tmpbuffer);
     
      TmpValueMax = GnuStatistic.getMaxSpeed();
      TmpValueMin = GnuStatistic.getMinSpeed();  
      sprintf(tmpbuffer,"V km/h : %3.0f / %3.0f", TmpValueMin, TmpValueMax );
      screen.setCursor(5, 145);
      screen.println(tmpbuffer);

      TmpValueMax = GnuStatistic.getMaxVario();
      TmpValueMin = GnuStatistic.getMinVario();  
      sprintf(tmpbuffer,"Vario : %2.1f / %2.1f", TmpValueMin, TmpValueMax );
      screen.setCursor(5, 175);
      screen.println(tmpbuffer);

      screen.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);

      return true;
    }   
  }  
  screen.fillScreen(GxEPD_WHITE); 
//  screen.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, true);
  varioScreen.setPage(0,true);  //force update  
  return false;  
}

/*--------------------------------------*/
/*                                      */
/*            soundPage                 */
/*                                      */
/*--------------------------------------*/

ScreenDigit soundDigit(screen, 100, 150, 2, 0, false, ALIGNNONE);
boolean state = false;

void soundPage(void) {

uint8_t volumeSound = GnuSettings.VARIOMETER_BEEP_VOLUME;


#ifdef SPEAKER
  toneAC_notone();
#endif //SPEAKER

#ifdef IMU_DEBUG
    Serial.println("Sound configuration");
#endif IMU_DEBUG
   
#ifdef HAVE_SCREEN

  screen.fillRect(0, 0, GxEPD_WIDTH-1, GxEPD_HEIGHT-1, GxEPD_WHITE);
  displaySound(volumeSound);

#ifdef IMU_DEBUG
       Serial.println("before boucle ");
#endif IMU_DEBUG

#endif //HAVE_SCREEN

  while (true) {

#ifdef IMU_DEBUG
//       Serial.println("boucle ");
#endif IMU_DEBUG

    if (stateLeftInterrup == HIGH) {
      delay(200);
      stateLeftInterrup = LOW;
      //Action - Left button press
      if (state == false) {
        
#ifdef IMU_DEBUG
       Serial.println("sortie boucle ");
#endif IMU_DEBUG

        break;     
      }
      else {
        if (volumeSound > 0) {
          volumeSound--;
          displaySound(volumeSound);
          toneAC(800,volumeSound);
          delay(1000);
          toneAC_notone();
       }
      }
    }   
    else if (stateRightInterrup == HIGH) {
        // check if the pushbutton is pressed.
       delay(200);
       stateRightInterrup = LOW;
      //Action - Right button press
      if (state == true) {
        // Sound +1
        if (volumeSound < 10) {
          volumeSound++;
          displaySound(volumeSound);
          toneAC(800,volumeSound);
          delay(1000);
          toneAC_notone();
        }
      }      
    }   
    else if (statePowerInt == HIGH) {
      delay(200);
      
      if (state) {
        
#ifdef IMU_DEBUG
         Serial.print("volumeSound = ");
         Serial.println(volumeSound);
#endif //IMU_DEBUG

          GnuSettings.soundSettingWrite(volumeSound);  
          volLevel.setVolume(GnuSettings.VARIOMETER_BEEP_VOLUME);
          beeper.setVolume(GnuSettings.VARIOMETER_BEEP_VOLUME);
      }
      else {
        toneAC(800,volumeSound);
        delay(1000);
        toneAC_notone();      
      }
      
      state = !state;   
      displaySound(volumeSound);    
      statePowerInt = LOW;
    }   
  }   
  screen.fillScreen(GxEPD_WHITE); 
//  screen.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, true);
  varioScreen.setPage(0,true);  //force update
}

void displaySound(int8_t volume) {
  if (volume == 0)   screen.drawBitmap(36, 0, sound_0, 128, 128, GxEPD_WHITE); 
  else if (volume < 5) screen.drawBitmap(36, 0, sound_1, 128, 128, GxEPD_WHITE); 
  else if (volume < 9) screen.drawBitmap(36, 0, sound_2, 128, 128, GxEPD_WHITE); 
  else  screen.drawBitmap(36, 0, sound_3, 128, 128, GxEPD_WHITE); 
  soundDigit.setValue(volume);
  soundDigit.display();
  if (state) {
    screen.fillRoundRect(50, 180, 100, 10, 0, GxEPD_BLACK);
  }
  else
  {
    screen.fillRoundRect(50, 180, 100, 10, 0, GxEPD_WHITE);    
  }
 // screen.update(); 
  screen.updateWindow(0, 0, GxEPD_WIDTH, GxEPD_HEIGHT, false);
}
