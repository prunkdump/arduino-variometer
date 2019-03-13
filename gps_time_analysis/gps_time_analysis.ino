#include <Arduino.h>
#include <SPI.h>
#include <VarioSettings.h>
#include <IntTW.h>
#include <vertaccel.h>
#include <EEPROM.h>
#include <LightInvensense.h>
#include <avr/pgmspace.h>
#include <varioscreen.h>
#include <digit.h>
#include <SdCard.h>
#include <LightFat16.h>
#include <SerialNmea.h>
#include <NmeaParser.h>
#include <LxnavSentence.h>
#include <LK8Sentence.h>
#include <IGCSentence.h>
#include <FirmwareUpdater.h>

/********************************************************/
/* This sketch give in order :                          */
/* -> The last GPS period                               */
/* -> The average GPS period                            */
/* -> The time needed to sent the RMC and GGA sentences */
/********************************************************/

/* where output ? */
//#define SERIAL_OUTPUT
#define SDCARD_OUTPUT

#define OUTPUT_PRECISION 1
#define MAX_SILENT_DURATION 50
#define PERIOD_MEAN_FILTER_SIZE 20


/***************/
/* IMU objects */
/***************/
Vertaccel vertaccel;


/*****************/
/* screen objets */
/*****************/
#ifdef HAVE_SCREEN
#define VARIOSCREEN_GPS_PERIOD_ANCHOR_X 62
#define VARIOSCREEN_GPS_PERIOD_ANCHOR_Y 0
#define VARIOSCREEN_GPS_MEAN_PERIOD_ANCHOR_X 62
#define VARIOSCREEN_GPS_MEAN_PERIOD_ANCHOR_Y 2
#define VARIOSCREEN_GPS_DURATION_ANCHOR_X 62
#define VARIOSCREEN_GPS_DURATION_ANCHOR_Y 4
#define VARIOSCREEN_SAT_ANCHOR_X 68
#define VARIOSCREEN_SAT_ANCHOR_Y 0

VarioScreen screen(VARIOSCREEN_DC_PIN,VARIOSCREEN_CS_PIN,VARIOSCREEN_RST_PIN);
ScreenDigit gpsPeriodDigit(screen, VARIOSCREEN_GPS_PERIOD_ANCHOR_X, VARIOSCREEN_GPS_PERIOD_ANCHOR_Y, OUTPUT_PRECISION, false);
ScreenDigit gpsMeanPeriodDigit(screen, VARIOSCREEN_GPS_MEAN_PERIOD_ANCHOR_X, VARIOSCREEN_GPS_MEAN_PERIOD_ANCHOR_Y, OUTPUT_PRECISION, false);
ScreenDigit gpsDurationDigit(screen, VARIOSCREEN_GPS_DURATION_ANCHOR_X, VARIOSCREEN_GPS_DURATION_ANCHOR_Y, OUTPUT_PRECISION, false);
SATLevel satLevel(screen, VARIOSCREEN_SAT_ANCHOR_X, VARIOSCREEN_SAT_ANCHOR_Y);

unsigned long lastSatLevelDisplay = 0;
#define SAT_LEVEL_DISPLAY_DELAY 1000

ScreenSchedulerObject displayList[] = { {&gpsPeriodDigit, 0}, {&gpsMeanPeriodDigit, 0}, {&gpsDurationDigit, 0}, {&satLevel, 0} };
ScreenScheduler varioScreen(screen, displayList, sizeof(displayList)/sizeof(ScreenSchedulerObject), 0, 0);
#endif //HAVE_SCREEN

/***************/
/* gps objects */
/***************/
NmeaParser nmeaParser;

/*******************/
/* SD card objects */
/*******************/
#if defined(HAVE_SDCARD) && defined(SDCARD_OUTPUT)

char filename[] = "TIMES00";
#define FILE_NAME_SIZE 7

lightfat16 file(SDCARD_CS_PIN);

#define SD_CARD_STATE_INITIAL 0
#define SD_CARD_STATE_INIT 1
#define SD_CARD_STATE_BEGIN 2
int sdcardState = SD_CARD_STATE_INITIAL; 

#endif //HAVE_SDCARD

#if (defined(HAVE_SDCARD) && defined(SDCARD_OUTPUT)) || defined(SERIAL_OUTPUT)
  Digit valueDigit;
#endif


/*******************/
/* period measures */
/*******************/
unsigned long gpsLastLockTimestamp;
unsigned long gpsLastReleaseTimestamp;
unsigned long periodDuration;
unsigned long lockDuration;
unsigned long lastPeriodDurations[PERIOD_MEAN_FILTER_SIZE];
int lastPeriodPos = 0;
double periodMean;
bool needOutput = false;


#if (defined(HAVE_SDCARD) && defined(SDCARD_OUTPUT)) || defined(SERIAL_OUTPUT)
void writeNumbers( double* values, int count ) {

#ifdef SERIAL_OUTPUT
  serialNmea.lock();
#endif

  /* write each number */
  for(int i = 0; i<count; i++) {

    valueDigit.begin(values[i], OUTPUT_PRECISION);
    while(valueDigit.available()) {

      uint8_t c = valueDigit.get();
#if defined(HAVE_SDCARD) && defined(SDCARD_OUTPUT)
      if( sdcardState == SD_CARD_STATE_BEGIN ) {
        file.write(c);
      }
#endif
#ifdef SERIAL_OUTPUT
      serialNmea.write(c);
#endif
    }
    
#if defined(HAVE_SDCARD) && defined(SDCARD_OUTPUT)
    if( sdcardState == SD_CARD_STATE_BEGIN ) {
      file.write('\n');
    }
#endif
#ifdef SERIAL_OUTPUT
    serialNmea.write('\n');
#endif
  }

#if defined(HAVE_SDCARD) && defined(SDCARD_OUTPUT)
  if( sdcardState == SD_CARD_STATE_BEGIN ) {
    file.write('\n');
  }
#endif
#ifdef SERIAL_OUTPUT
  serialNmea.write('\n');
  serialNmea.release();
#endif

}
#endif


void computeMeanPeriod(void) {

  /* save new value */
  lastPeriodDurations[lastPeriodPos] = periodDuration;
  lastPeriodPos = (lastPeriodPos + 1) % PERIOD_MEAN_FILTER_SIZE;

  /* compute mean */
  unsigned long periodSum = 0;
  for( int i = 0; i<PERIOD_MEAN_FILTER_SIZE; i++) {
    periodSum += lastPeriodDurations[i];
  }

  periodMean = (double)periodSum/(double)PERIOD_MEAN_FILTER_SIZE;
}


bool gpsCheckLock(void) {

  /* save times */
  unsigned long receiveTimestamp = serialNmea.getReceiveTimestamp();
  
  if( gpsLastLockTimestamp != receiveTimestamp ) {
    periodDuration = receiveTimestamp - gpsLastLockTimestamp;
    lockDuration = gpsLastReleaseTimestamp - gpsLastLockTimestamp;
    gpsLastLockTimestamp = receiveTimestamp;
    gpsLastReleaseTimestamp = gpsLastLockTimestamp;
    computeMeanPeriod();
    return true;
  } 
  return false;
}


void gpsSaveRelease(void) {
  gpsLastReleaseTimestamp = millis();
}

#if (defined(HAVE_SDCARD) && defined(SDCARD_OUTPUT)) || defined(SERIAL_OUTPUT)
void outputResults(void) {

  /* sdcard/serial output*/
#if defined(HAVE_SDCARD) && defined(SDCARD_OUTPUT)
  /* sdcard output */
  if( sdcardState > SD_CARD_STATE_INITIAL) {

    /* create file */
    if( sdcardState == SD_CARD_STATE_INIT) {
      file.init();
      file.begin(filename, FILE_NAME_SIZE);
      sdcardState = SD_CARD_STATE_BEGIN;
    }
  }
#endif

#if defined(SERIAL_OUTPUT) || defined(SDCARD_OUTPUT)
  double outputValues[] = {(double)periodDuration, periodMean, (double)lockDuration};
  writeNumbers(outputValues, 3);
#endif
}   
#endif

 
void setup() {

  /*****************************/
  /* wait for devices power on */
  /*****************************/
  delay(VARIOMETER_POWER_ON_DELAY);

  /**********************/
  /* init accelerometer */
  /**********************/
  intTW.begin();
#ifdef HAVE_ACCELEROMETER
  vertaccel.init();
  if( firmwareUpdateCond() ) {
   firmwareUpdate();
  }
#endif //HAVE_ACCELEROMETER

  /************/
  /* init SPI */
  /************/ 

  /* set all SPI CS lines before talking to devices */
#if defined(HAVE_SDCARD) && defined(SDCARD_OUTPUT)
  file.enableSPI();
#endif //HAVE_SDCARD

#ifdef HAVE_SCREEN
  screen.enableSPI();
#endif //HAVE_SCREEN

  /****************/
  /* init SD Card */
  /****************/
#if defined(HAVE_SDCARD) && defined(SDCARD_OUTPUT)
  if( file.init() >= 0 ) {
    sdcardState = SD_CARD_STATE_INIT;
  }
#endif HAVE_SDCARD
 
  /***************/
  /* init screen */
  /***************/
#ifdef HAVE_SCREEN
  screen.begin(VARIOSCREEN_CONTRAST);
#endif //HAVE_SCREEN
  
  /************/
  /* init gps */
  /************/
  serialNmea.begin(GPS_BLUETOOTH_BAUDS, true);

}

void loop() {

  if( gpsCheckLock() ) {
    
    /* screen output */
    gpsPeriodDigit.setValue((double)periodDuration);
    gpsMeanPeriodDigit.setValue(periodMean);
    gpsDurationDigit.setValue((double)lockDuration);

    /* serial/sdcard output later */
    /* as serial output is currently in use */
    needOutput = true;
    
  }

#if (defined(HAVE_SDCARD) && defined(SDCARD_OUTPUT)) || defined(SERIAL_OUTPUT)
  if( millis() - serialNmea.getLastReceiveTimestamp() > MAX_SILENT_DURATION ) {
    outputResults();
    needOutput = false;
  }
#endif

  /* parse when possible and check when release */
  if( serialNmea.lockRMC() ) {
    nmeaParser.beginRMC();
  }

  if( serialNmea.lockGGA() ) {
    nmeaParser.beginGGA();
  }

  if( nmeaParser.isParsing() ) {
    while( nmeaParser.isParsing() ) {
      uint8_t c = serialNmea.read();
        
      /* parse sentence */        
      nmeaParser.feed( c );
    }

    serialNmea.release();
    gpsSaveRelease();
  }

#ifdef HAVE_SCREEN
  /* screen update */
  varioScreen.displayStep();
  if( millis() - lastSatLevelDisplay > SAT_LEVEL_DISPLAY_DELAY ) {
    lastSatLevelDisplay = millis();
    satLevel.setSatelliteCount( nmeaParser.satelliteCount );
  }
#endif
}
