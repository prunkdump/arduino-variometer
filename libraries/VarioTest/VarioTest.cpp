#include <VarioTest.h>

#include <VarioSettings.h>
#include <I2Cdev.h>
#include <ms5611.h>
#include <vertaccel.h>
#include <EEPROM.h>
#include <LightInvensense.h>
#include <kalmanvert.h>
#include <beeper.h>
#include <toneAC.h>
#include <avr/pgmspace.h>
#include <varioscreen.h>
#include <digit.h>
#include <SdCard.h>
#include <LightFat16.h>
#include <SerialNmea.h>
#include <string.h>
#include <FlightHistory.h>

/*******************/
/* Version         */
/*******************/

#define VERSION 1
#define SUB_VERSION 0

#define BEEP_FREQ 800

#ifdef HAVE_SDCARD
lightfat16 file(SDCARD_CS_PIN);

#define SDCARD_STATE_INITIAL 0
#define SDCARD_STATE_INITIALIZED 1
#define SDCARD_STATE_READY 2
#define SDCARD_STATE_ERROR -1
#endif //HAVE_SDCARD

/*****************/
/* screen objets */
/*****************/
#ifdef HAVE_SCREEN

unsigned long lastLowFreqUpdate = 0;

#ifdef HAVE_GPS 
#define VARIOSCREEN_ALTI_ANCHOR_X 50
#define VARIOSCREEN_ALTI_ANCHOR_Y 0
#define VARIOSCREEN_VARIO_ANCHOR_X 40
#define VARIOSCREEN_VARIO_ANCHOR_Y 2
#define VARIOSCREEN_TIME_ANCHOR_X 38
#define VARIOSCREEN_TIME_ANCHOR_Y 4
#define VARIOSCREEN_ELAPSED_TIME_ANCHOR_X 38
#define VARIOSCREEN_ELAPSED_TIME_ANCHOR_Y 4
#define VARIOSCREEN_SPEED_ANCHOR_X 22
#define VARIOSCREEN_SPEED_ANCHOR_Y 4
#define VARIOSCREEN_GR_ANCHOR_X 84
#define VARIOSCREEN_GR_ANCHOR_Y 2
#define VARIOSCREEN_TREND_ANCHOR_X 48
#define VARIOSCREEN_TREND_ANCHOR_Y 2
#define RATIO_MAX_VALUE 30.0
#define RATIO_MIN_SPEED 10.0
#else
#define VARIOSCREEN_ALTI_ANCHOR_X 52
#define VARIOSCREEN_ALTI_ANCHOR_Y 1
#define VARIOSCREEN_VARIO_ANCHOR_X 52
#define VARIOSCREEN_VARIO_ANCHOR_Y 3
#endif //HAVE_GPS

#define VARIOSCREEN_BAT_ANCHOR_X 68
#define VARIOSCREEN_BAT_ANCHOR_Y 1
#define VARIOSCREEN_SAT_ANCHOR_X 68
#define VARIOSCREEN_SAT_ANCHOR_Y 0

#define VARIOSCREEN_RECORD_ANCHOR_X 2
#define VARIOSCREEN_RECORD_ANCHOR_Y 0

#define VARIOSCREEN_MUTE_ANCHOR_X 0
#define VARIOSCREEN_MUTE_ANCHOR_Y 1

VarioScreen screen(VARIOSCREEN_DC_PIN,VARIOSCREEN_CS_PIN,VARIOSCREEN_RST_PIN);
MSUnit msunit(screen, VARIOSCREEN_VARIO_ANCHOR_X, VARIOSCREEN_VARIO_ANCHOR_Y);
MUnit munit(screen, VARIOSCREEN_ALTI_ANCHOR_X, VARIOSCREEN_ALTI_ANCHOR_Y);
ScreenDigit altiDigit(screen, VARIOSCREEN_ALTI_ANCHOR_X, VARIOSCREEN_ALTI_ANCHOR_Y, 0, false);
ScreenDigit varioDigit(screen, VARIOSCREEN_VARIO_ANCHOR_X, VARIOSCREEN_VARIO_ANCHOR_Y, 1, true);
#ifdef HAVE_GPS
ScreenDigit speedDigit(screen, VARIOSCREEN_SPEED_ANCHOR_X, VARIOSCREEN_SPEED_ANCHOR_Y, 0, false);
ScreenDigit ratioDigit(screen, VARIOSCREEN_GR_ANCHOR_X, VARIOSCREEN_GR_ANCHOR_Y, 0, false);
KMHUnit kmhunit(screen, VARIOSCREEN_SPEED_ANCHOR_X, VARIOSCREEN_SPEED_ANCHOR_Y);
GRUnit grunit(screen, VARIOSCREEN_GR_ANCHOR_X, VARIOSCREEN_GR_ANCHOR_Y);
SATLevel satLevel(screen, VARIOSCREEN_SAT_ANCHOR_X, VARIOSCREEN_SAT_ANCHOR_Y);
RECORDIndicator recordIndicator(screen, VARIOSCREEN_RECORD_ANCHOR_X, VARIOSCREEN_RECORD_ANCHOR_Y);
TRENDLevel trendLevel(screen, VARIOSCREEN_TREND_ANCHOR_X, VARIOSCREEN_TREND_ANCHOR_Y);

ScreenTime screenTime(screen, VARIOSCREEN_TIME_ANCHOR_X, VARIOSCREEN_TIME_ANCHOR_Y);
ScreenElapsedTime screenElapsedTime(screen, VARIOSCREEN_ELAPSED_TIME_ANCHOR_X, VARIOSCREEN_ELAPSED_TIME_ANCHOR_Y);
#endif //HAVE_GPS
#ifdef HAVE_ACCELEROMETER
ScreenMuteIndicator muteIndicator(screen, VARIOSCREEN_MUTE_ANCHOR_X, VARIOSCREEN_MUTE_ANCHOR_Y);
#endif //HAVE_ACCELEROMETER
#ifdef HAVE_VOLTAGE_DIVISOR
BATLevel batLevel(screen, VARIOSCREEN_BAT_ANCHOR_X, VARIOSCREEN_BAT_ANCHOR_Y, VOLTAGE_DIVISOR_VALUE, VOLTAGE_DIVISOR_REF_VOLTAGE);
int maxVoltage = 0;
#endif //HAVE_VOLTAGE_DIVISOR


ScreenSchedulerObject displayList[] = { {&msunit, 0}, {&munit, 0}, {&altiDigit, 0}, {&varioDigit, 0}
#ifdef HAVE_GPS
  									   ,{&kmhunit, 0}, {&speedDigit, 0}, {&ratioDigit, 0}, {&satLevel, 0}, {&screenTime, 1}, {&screenElapsedTime, 0}, {&recordIndicator, 0} , {&trendLevel, 0}
#endif //HAVE_GPS
#ifdef HAVE_ACCELEROMETER
                                       ,{&muteIndicator, 0}
#endif //HAVE_ACCELEROMETER
#ifdef HAVE_VOLTAGE_DIVISOR
                                       , {&batLevel, 0}
#endif //HAVE_VOLTAGE_DIVISOR
};

#ifdef HAVE_GPS
ScreenScheduler varioScreen(screen, displayList, sizeof(displayList)/sizeof(ScreenSchedulerObject), 0, 0);
bool DisplayDuration;
#else
ScreenScheduler varioScreen(screen, displayList, sizeof(displayList)/sizeof(ScreenSchedulerObject), 0, 0);
#endif //HAVE_GPS

#endif //HAVE_SCREEN

int8_t sdcardState = SDCARD_STATE_INITIAL;

const byte BUFFER_SIZE = 64;
 
/* Variables d'exemple qui seront chargé depuis le fichier de configuration */
int sensorValue;  
 
/**********************/
/* alti/vario objects */
/**********************/
#define POSITION_MEASURE_STANDARD_DEVIATION 0.1
#ifdef HAVE_ACCELEROMETER 
#define ACCELERATION_MEASURE_STANDARD_DEVIATION 0.3
#else
#define ACCELERATION_MEASURE_STANDARD_DEVIATION 0.6
#endif //HAVE_ACCELEROMETER 

kalmanvert kalmanvert;
 
#ifdef HAVE_SPEAKER
beeper beeper(VARIOMETER_SINKING_THRESHOLD, VARIOMETER_CLIMBING_THRESHOLD, VARIOMETER_NEAR_CLIMBING_SENSITIVITY, VARIOMETER_BEEP_VOLUME);
#define BEEP_FREQ 800
#endif
 
/************************************/
/* glide ratio / average climb rate */
/************************************/
#if defined(HAVE_GPS) || defined(VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE) || (RATIO_CLIMB_RATE > 1)

/* determine history params */
#ifdef HAVE_GPS
#if defined(VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE) || (RATIO_CLIMB_RATE > 1)
/* unsure period divide GPS_PERIOD */
const double historyGPSPeriodCountF = (double)(VARIOMETER_INTEGRATION_DISPLAY_FREQ)*(double)(GPS_PERIOD)/(1000.0);
const int8_t historyGPSPeriodCount = (int8_t)(0.5 + historyGPSPeriodCountF);

const double historyPeriodF = (double)(GPS_PERIOD)/(double)(historyGPSPeriodCount);
const unsigned historyPeriod = (unsigned)(0.5 + historyPeriodF);
#else
/* GPS give the period */
const int8_t historyGPSPeriodCount = 1;
const unsigned historyPeriod = GPS_PERIOD;
#endif //VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE || (RATIO_CLIMB_RATE > 1)

const double historyCountF = (double)(VARIOMETER_INTEGRATION_TIME)/(double)historyPeriod;
const int8_t historyCount = (int8_t)(0.5 + historyCountF);
#else
const double historyCountF = (double)(VARIOMETER_INTEGRATION_DISPLAY_FREQ)*(double)(VARIOMETER_INTEGRATION_TIME)/(1000.0);
const int8_t historyCount = (int8_t)(0.5 + historyCountF);

const double historyPeriodF = (double)(VARIOMETER_INTEGRATION_TIME)/(double)historyCount;
const unsigned historyPeriod = (unsigned)(0.5 + historyPeriodF);
#endif //HAVE_GPS
 
/* create history */
#ifdef HAVE_GPS
SpeedFlightHistory<historyPeriod, historyCount, historyGPSPeriodCount> history;
#else
FlightHistory<historyPeriod, historyCount> history;
#endif

void VarioTest::Test(int Number) {
  for( int i = 0; i<Number; i++) {
    toneAC(BEEP_FREQ);
    delay(200);
    toneAC(0);
    delay(200);
  }	
	
  switch (Number) {
    case 1:    
	  if (TestSound() == false) ErrorSound();
	  else 						TestOK();
	  break;
    case 2:    
	  if (TestSDCARD() == false) ErrorSound();
	  else 						TestOK();
	  break;
    case 3:    
	  if (TestScreen() == false) ErrorSound();
	  else 						TestOK();
	  break;
	case 10:
//	  TestEND();
	  break;
  }
  
  delay(2000);
//  TestOK();  
}

bool VarioTest::TestSound() {
  for (int i=0;i<20;i++) {
    toneAC(100 * i);
    delay(100);
    toneAC(0);
  }  	
  return true;
}

void VarioTest::ErrorSound() {
#if defined( HAVE_SPEAKER)
  for (int i=0;i<3;i++) {
    toneAC(300);
    delay(2000);
    toneAC(0);
  }
#endif //HAVE_SPEAKER
  for(;;);	  
}

void VarioTest::TestOK() {
#if defined( HAVE_SPEAKER)
  for (int i=0;i<2;i++) {
    toneAC(6000);
    delay(300);
    toneAC(0);
  }
#endif //HAVE_SPEAKER
}

void VarioTest::TestEND() {
#if defined( HAVE_SPEAKER)
  for (int i=0;i<2;i++) {
    toneAC(BEEP_FREQ);
    delay(1000);
    toneAC(0);
  }
#endif //HAVE_SPEAKER
}

bool VarioTest::TestSDCARD() {
	
    /* Déclare le buffer qui stockera une ligne du fichier, ainsi que les deux pointeurs key et value */
  char buffer[BUFFER_SIZE], *key, *value;
 
  /* Déclare l'itérateur et le compteur de lignes */
  byte i, buffer_lenght, line_counter = 0;
 
  /* set all SPI CS lines before talking to devices */
#if defined(HAVE_SDCARD)
  file.enableSPI();
#endif //defined(HAVE_SDCARD) 

  
  /****************/
  /* init SD Card */
  /****************/
#if defined(HAVE_SDCARD)
  if( file.init() >= 0 ) {
    sdcardState = SDCARD_STATE_INITIALIZED;  //useless to set error
  }
  else
  {
	return false;
  }  
#endif //defined(HAVE_SDCARD)
  
  /* Ouvre le  fichier de configuration */
  
      delay(1000);


      /* build date */
    uint8_t dateChar[8]; //two bytes are used for incrementing number on filename
    for(int i=0; i<6; i++) {
      dateChar[i] = 'A';
    }

    /* create file */    
    if( file.begin((char*)dateChar, 8) >= 0 ) {
      sdcardState = SDCARD_STATE_READY;
    }
    else {
	  return false;
    }
 
    delay(1000);

	for (int j=0;j<26;j++) {
      for (int i=1;i<100;i++) {
        file.write( char(65+j) );
      }
	}
	return true;
}

bool VarioTest::TestScreen() {

  /***************/
  /* init screen */
  /***************/
#ifdef HAVE_SCREEN
  screen.begin(VARIOSCREEN_CONTRAST);
 
  unsigned long timer = millis();
  int8_t tmptime[] = {0,SUB_VERSION,VERSION};
  screenTime.setTime(tmptime);
 
  varioScreen.setPage(1);
  DisplayDuration = true;
  screenTime.update();
#endif //Screen
  
}  

bool VarioTest::TestMS5611() {

  /******************/
  /* init barometer */
  /******************/
   ms5611_init();

   while (true) {

  /*****************************/
  /* compute vertical velocity */
  /*****************************/
#ifdef HAVE_ACCELEROMETER
		if( ms5611_dataReady() && vertaccel.dataReady() ) {
			ms5611_updateData();
    
			kalmanvert.update( ms5611_getAltitude(),
							   vertaccel.getValue(),
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

			double currentalti  = kalmanvert.getCalibratedPosition();
			double currentvario = kalmanvert.getVelocity();

#ifdef HAVE_SCREEN
			altiDigit.setValue(currentalti);
			varioDigit.setValue(currentvario);
#endif //HAVE_SCREEN
     
		}

  /*****************/
  /* update beeper */
  /*****************/
#ifdef HAVE_SPEAKER
		beeper.update();
#endif //HAVE_SPEAKER

#ifdef HAVE_SCREEN
		varioScreen.setPage(0);
#endif //HAVE_SCREEN
	}

  /* screen update */
#ifdef HAVE_SCREEN
	varioScreen.displayStep();
#endif //HAVE_SCREEN
  }
}

#endif

/*  
#ifdef HAVE_VOLTAGE_DIVISOR
      /* update battery level *
      batLevel.setVoltage( analogRead(VOLTAGE_DIVISOR_PIN) );
      batLevel.update();
#endif //HAVE_VOLTAGE_DIVISOR

  altiDigit.setValue(flystat.GetAlti());
  altiDigit.update();
  varioDigit.setValue(flystat.GetVarioMin());
  varioDigit.update();
  double tmpspeed = flystat.GetSpeed();
  if (tmpspeed > 99) tmpspeed = 99;
  speedDigit.setValue(tmpspeed);
  speedDigit.update();
  kmhunit.update();
  msunit.update();
  munit.update();  

 #endif HAVE_SCREEN_JPG63
 #endif //HAVE_SCREEN
}
*/