/* variometer -- The GNUVario embedded code
 *
 * Copyright 2016-2019 Baptiste PELLEGRIN
 * 
 * This file is part of GNUVario.
 *
 * GNUVario is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNUVario is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <Arduino.h>
#include <SPI.h>
#include <VarioSettings.h>
#include <IntTW.h>
#include <ms5611.h>
#include <vertaccel.h>
#include <EEPROM.h>
#include <LightInvensense.h>
#include <TwoWireScheduler.h>
#include <kalmanvert.h>
#include <beeper.h>
#include <toneAC.h>
#include <avr/pgmspace.h>
#include <varioscreen.h>
#include <digit.h>
#include <LightSdCard.h>
#include <LightFat16.h>
#include <SerialNmea.h>
#include <NmeaParser.h>
#include <LxnavSentence.h>
#include <LK8Sentence.h>
#include <IGCSentence.h>
#include <FirmwareUpdaterTWS.h>
#include <FlightHistory.h>

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*!!            !!! WARNING  !!!              !!*/
/*!! Before building check :                  !!*/
/*!! libraries/VarioSettings/VarioSettings.h  !!*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/


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

/***************/
/* IMU objects */
/***************/
#ifdef HAVE_BMP280
Bmp280 TWScheduler::bmp280;
#else
Ms5611 TWScheduler::ms5611;
#endif
#ifdef HAVE_ACCELEROMETER
Vertaccel TWScheduler::vertaccel;
#endif


/*****************/
/* screen objets */
/*****************/
#ifdef HAVE_SCREEN

unsigned long lastLowFreqUpdate = 0;

#ifdef HAVE_GPS 
#define VARIOSCREEN_ALTI_ANCHOR_X 52
#define VARIOSCREEN_ALTI_ANCHOR_Y 0
#define VARIOSCREEN_VARIO_ANCHOR_X 52
#define VARIOSCREEN_VARIO_ANCHOR_Y 2
#define VARIOSCREEN_TIME_ANCHOR_X 5
#define VARIOSCREEN_TIME_ANCHOR_Y 0
#define VARIOSCREEN_ELAPSED_TIME_ANCHOR_X 5
#define VARIOSCREEN_ELAPSED_TIME_ANCHOR_Y 3
#define VARIOSCREEN_SPEED_ANCHOR_X 22
#define VARIOSCREEN_SPEED_ANCHOR_Y 4
#define VARIOSCREEN_GR_ANCHOR_X 72
#define VARIOSCREEN_GR_ANCHOR_Y 4
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

#define VARIOSCREEN_MUTE_ANCHOR_X 2
#define VARIOSCREEN_MUTE_ANCHOR_Y 0

VarioScreen screen(VARIOSCREEN_DC_PIN,VARIOSCREEN_CS_PIN,VARIOSCREEN_RST_PIN);
MSUnit msunit(screen, VARIOSCREEN_VARIO_ANCHOR_X, VARIOSCREEN_VARIO_ANCHOR_Y);
MUnit munit(screen, VARIOSCREEN_ALTI_ANCHOR_X, VARIOSCREEN_ALTI_ANCHOR_Y);
ScreenDigit altiDigit(screen, VARIOSCREEN_ALTI_ANCHOR_X, VARIOSCREEN_ALTI_ANCHOR_Y, 0, false);
ScreenDigit varioDigit(screen, VARIOSCREEN_VARIO_ANCHOR_X, VARIOSCREEN_VARIO_ANCHOR_Y, 1, true);
#ifdef HAVE_GPS
ScreenDigit speedDigit(screen, VARIOSCREEN_SPEED_ANCHOR_X, VARIOSCREEN_SPEED_ANCHOR_Y, 0, false);
ScreenDigit ratioDigit(screen, VARIOSCREEN_GR_ANCHOR_X, VARIOSCREEN_GR_ANCHOR_Y, 1, false);
KMHUnit kmhunit(screen, VARIOSCREEN_SPEED_ANCHOR_X, VARIOSCREEN_SPEED_ANCHOR_Y);
GRUnit grunit(screen, VARIOSCREEN_GR_ANCHOR_X, VARIOSCREEN_GR_ANCHOR_Y);
SATLevel satLevel(screen, VARIOSCREEN_SAT_ANCHOR_X, VARIOSCREEN_SAT_ANCHOR_Y);
ScreenTime screenTime(screen, VARIOSCREEN_TIME_ANCHOR_X, VARIOSCREEN_TIME_ANCHOR_Y);
ScreenElapsedTime screenElapsedTime(screen, VARIOSCREEN_ELAPSED_TIME_ANCHOR_X, VARIOSCREEN_ELAPSED_TIME_ANCHOR_Y);
#endif //HAVE_GPS
#ifdef HAVE_ACCELEROMETER
ScreenMuteIndicator muteIndicator(screen, VARIOSCREEN_MUTE_ANCHOR_X, VARIOSCREEN_MUTE_ANCHOR_Y);
#endif //HAVE_ACCELEROMETER
#ifdef HAVE_VOLTAGE_DIVISOR
BATLevel batLevel(screen, VARIOSCREEN_BAT_ANCHOR_X, VARIOSCREEN_BAT_ANCHOR_Y, VOLTAGE_DIVISOR_VALUE, VOLTAGE_DIVISOR_REF_VOLTAGE);
#endif //HAVE_VOLTAGE_DIVISOR


ScreenSchedulerObject displayList[] = { {&msunit, 0}, {&munit, 0}, {&altiDigit, 0}, {&varioDigit, 0}
#ifdef HAVE_GPS
                                       ,{&kmhunit, 0}, {&grunit, 0}, {&speedDigit, 0}, {&ratioDigit, 0}, {&satLevel, 0}, {&screenTime, 1}, {&screenElapsedTime, 1}
#endif //HAVE_GPS
#ifdef HAVE_ACCELEROMETER
                                       ,{&muteIndicator, 0}
#endif //HAVE_ACCELEROMETER
#ifdef HAVE_VOLTAGE_DIVISOR
                                       , {&batLevel, 0}
#endif //HAVE_VOLTAGE_DIVISOR
};

#ifdef HAVE_GPS
ScreenScheduler varioScreen(screen, displayList, sizeof(displayList)/sizeof(ScreenSchedulerObject), 0, 1);
#else
ScreenScheduler varioScreen(screen, displayList, sizeof(displayList)/sizeof(ScreenSchedulerObject), 0, 0);
#endif //HAVE_GPS

#endif //HAVE_SCREEN

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
#endif

/************************************/
/* glide ratio / average climb rate */
/************************************/
#if defined(HAVE_GPS) || defined(VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE)

/* determine history params */
#ifdef HAVE_GPS
#ifdef VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE
/* unsure period divide GPS_PERIOD */
constexpr double historyGPSPeriodCountF = (double)(VARIOMETER_INTEGRATED_CLIMB_RATE_DISPLAY_FREQ)*(double)(GPS_PERIOD)/(1000.0);
constexpr int8_t historyGPSPeriodCount = (int8_t)(0.5 + historyGPSPeriodCountF);

constexpr double historyPeriodF = (double)(GPS_PERIOD)/(double)(historyGPSPeriodCount);
constexpr unsigned historyPeriod = (unsigned)(0.5 + historyPeriodF);
#else
/* GPS give the period */
constexpr int8_t historyGPSPeriodCount = 1;
constexpr unsigned historyPeriod = GPS_PERIOD;
#endif //VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE

constexpr double historyCountF = (double)(VARIOMETER_GLIDE_RATIO_INTEGRATION_TIME)/(double)historyPeriod;
constexpr int8_t historyCount = (int8_t)(0.5 + historyCountF);
#else //!HAVE_GPS
constexpr double historyCountF = (double)(VARIOMETER_INTEGRATED_CLIMB_RATE_DISPLAY_FREQ)*(double)(VARIOMETER_CLIMB_RATE_INTEGRATION_TIME)/(1000.0);
constexpr int8_t historyCount = (int8_t)(0.5 + historyCountF);

constexpr double historyPeriodF = (double)(VARIOMETER_CLIMB_RATE_INTEGRATION_TIME)/(double)historyCount;
constexpr unsigned historyPeriod = (unsigned)(0.5 + historyPeriodF);
#endif //HAVE_GPS

/* create history */
#ifdef HAVE_GPS
SpeedFlightHistory<historyPeriod, historyCount, historyGPSPeriodCount> history;
#else
#ifdef VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE
FlightHistory<historyPeriod, historyCount> history;
#endif //VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE
#endif //HAVE_GPS

/* compute climb rate period count when differ from glide ratio period count */
#if defined(HAVE_GPS) && defined(VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE)
#if VARIOMETER_CLIMB_RATE_INTEGRATION_TIME > VARIOMETER_GLIDE_RATIO_INTEGRATION_TIME
#error VARIOMETER_CLIMB_RATE_INTEGRATION_TIME must be less or equal than VARIOMETER_GLIDE_RATIO_INTEGRATION_TIME
#endif
constexpr double historyClimbRatePeriodCountF = (double)(VARIOMETER_CLIMB_RATE_INTEGRATION_TIME)/(double)historyPeriod;
constexpr int8_t historyClimbRatePeriodCount = (int8_t)historyClimbRatePeriodCountF;
#endif

#endif //defined(HAVE_GPS) || defined(VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE)

/***************/
/* gps objects */
/***************/
#ifdef HAVE_GPS

NmeaParser nmeaParser;

#ifdef HAVE_BLUETOOTH
boolean lastSentence = false;
#endif //HAVE_BLUETOOTH

#ifdef HAVE_SDCARD
lightfat16 file;
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
#error No bluetooth sentence type specified !
#endif

#ifndef HAVE_GPS
unsigned long lastVarioSentenceTimestamp = 0;
#endif // !HAVE_GPS
#endif //HAVE_BLUETOOTH

#if defined(HAVE_ACCELEROMETER) && defined(HAVE_SPEAKER) 
/* tap callback : mute/unmute beeper */
void beeperTapCallback(unsigned char direction, unsigned char count) { 

  static bool muted = false;
  muted = !muted;
  toneACMute(muted);
#ifdef HAVE_SCREEN
  muteIndicator.setMuteState(muted);
#endif //HAVE_SCREEN
}
#endif //defined(HAVE_ACCELEROMETER) && defined(HAVE_SPEAKER) 


/*********************/
/* Low voltage beep  */
/*********************/
#ifdef HAVE_LOW_BATTERY_BEEP
unsigned long batteryBeeps_lastrun;
#endi

/*-----------------*/
/*      SETUP      */
/*-----------------*/
void setup() {

  /*****************************/
  /* wait for devices power on */
  /*****************************/
  delay(VARIOMETER_POWER_ON_DELAY);

  /**************************/
  /* init Two Wires devices */
  /**************************/
  intTW.begin();
  twScheduler.init();
#ifdef HAVE_ACCELEROMETER
  if( firmwareUpdateCondTWS() ) {
   firmwareUpdate();
  }
  fastMPUSetTapCallback(beeperTapCallback);
#endif //HAVE_ACCELEROMETER

  /************/
  /* init SPI */
  /************/ 

  /* set all SPI CS lines before talking to devices */
#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
  file.enableSPI();
#endif //defined(HAVE_SDCARD) && defined(HAVE_GPS)

#ifdef HAVE_SCREEN
  screen.enableSPI();
#endif //HAVE_SCREEN

  /****************/
  /* init SD Card */
  /****************/
#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
  if( file.init() >= 0 ) {
    sdcardState = SDCARD_STATE_INITIALIZED;  //useless to set error
  }
#endif //defined(HAVE_SDCARD) && defined(HAVE_GPS)
 
  /***************/
  /* init screen */
  /***************/
#ifdef HAVE_SCREEN
  screen.begin(VARIOSCREEN_CONTRAST);
#endif //HAVE_SCREEN
  
  /**************************/
  /* init gps and bluetooth */
  /**************************/
#if defined(HAVE_BLUETOOTH) || defined(HAVE_GPS)
#ifdef HAVE_GPS
  serialNmea.begin(GPS_BLUETOOTH_BAUDS, true);
#else
  serialNmea.begin(GPS_BLUETOOTH_BAUDS, false);
#endif //HAVE_GPS
#endif //defined(HAVE_BLUETOOTH) || defined(HAVE_GPS)
  
  /******************/
  /* get first data */
  /******************/
  
  /* wait for first alti */
  while( ! twScheduler.havePressure() ) { }
  
  /* init kalman filter with 0.0 accel */
  double firstAlti = twScheduler.getAlti();
  kalmanvert.init(firstAlti,
                  0.0,
                  POSITION_MEASURE_STANDARD_DEVIATION,
                  ACCELERATION_MEASURE_STANDARD_DEVIATION,
                  millis());
                  
  /* init history */
#if defined(HAVE_GPS) || defined(VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE)
  history.init(firstAlti, millis());
#endif //defined(HAVE_GPS) || defined(VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE)
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
  if( twScheduler.havePressure() && twScheduler.haveAccel() ) {
    kalmanvert.update( twScheduler.getAlti(),
                       twScheduler.getAccel(NULL),
                       millis() );
#else
  if( twScheduler.havePressure() ) {
    kalmanvert.update( twScheduler.getAlti(),
                       0.0,
                       millis() );
#endif //HAVE_ACCELEROMETER

    /* set beeper */
#ifdef HAVE_SPEAKER
    beeper.setVelocity( kalmanvert.getVelocity() );
#endif //HAVE_SPEAKER

    /* set history */
#if defined(HAVE_GPS) || defined(VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE)
    history.setAlti(kalmanvert.getCalibratedPosition(), millis());
#endif

    /* set screen */
#ifdef HAVE_SCREEN
    altiDigit.setValue(kalmanvert.getCalibratedPosition());
#ifdef VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE
    if( history.haveNewClimbRate() ) {
#ifdef HAVE_GPS
      varioDigit.setValue(history.getClimbRate(historyClimbRatePeriodCount)); //climb rate period can differ from glide ratio period
#else //!HAVE_GPS
      varioDigit.setValue(history.getClimbRate());
#endif //HAVE_GPS
    }
#else //!VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE
    varioDigit.setValue(kalmanvert.getVelocity());
#endif //VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE
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
          double gpsAlti = nmeaParser.getAlti();
          kalmanvert.calibratePosition(gpsAlti);
#if defined(HAVE_GPS) || defined(VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE)
          history.init(gpsAlti, millis());
#endif //defined(HAVE_GPS) || defined(VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE)

          variometerState = VARIOMETER_STATE_CALIBRATED;
#if defined(HAVE_SDCARD) && ! defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)
          createSDCardTrackFile();
#endif //defined(HAVE_SDCARD) && ! defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)
        }
      }
      
      /* else check if the flight have started */
      else {  //variometerState == VARIOMETER_STATE_CALIBRATED
        
        /* check flight start condition */
        if( (millis() > FLIGHT_START_MIN_TIMESTAMP) &&
            (kalmanvert.getVelocity() < FLIGHT_START_VARIO_LOW_THRESHOLD || kalmanvert.getVelocity() > FLIGHT_START_VARIO_HIGH_THRESHOLD) &&
            (nmeaParser.getSpeed() > FLIGHT_START_MIN_SPEED) ) {
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
    if( (millis() > FLIGHT_START_MIN_TIMESTAMP) &&
        (kalmanvert.getVelocity() < FLIGHT_START_VARIO_LOW_THRESHOLD || kalmanvert.getVelocity() > FLIGHT_START_VARIO_HIGH_THRESHOLD) ) {
      variometerState = VARIOMETER_STATE_FLIGHT_STARTED;
      enableflightStartComponents();
    }
  }
#endif // !HAVE_GPS

  /**********************************/
  /* update low freq screen objects */
  /**********************************/
#ifdef HAVE_SCREEN
  unsigned long lowFreqDuration = millis() - lastLowFreqUpdate;
  if( varioScreen.getPage() == 0 ) {
    if( lowFreqDuration > VARIOMETER_BASE_PAGE_DURATION ) {
#ifdef HAVE_GPS //no multipage without GPS
      varioScreen.nextPage();
    }
  } else { //page == 1
    if( lowFreqDuration > VARIOMETER_BASE_PAGE_DURATION + VARIOMETER_ALTERNATE_PAGE_DURATION ) {
#endif //HAVE_GPS multipage support
      lastLowFreqUpdate = millis();

#ifdef HAVE_GPS
      /* set time */
      screenTime.setTime( nmeaParser.time );
      screenTime.correctTimeZone( VARIOMETER_TIME_ZONE );
      screenElapsedTime.setCurrentTime( screenTime.getTime() );

      /* update satelite count */
      satLevel.setSatelliteCount( nmeaParser.satelliteCount );
#endif //HAVE_GPS  

#ifdef HAVE_VOLTAGE_DIVISOR
      /* update battery level */
      batLevel.setVoltage( analogRead(VOLTAGE_DIVISOR_PIN) );
#endif //HAVE_VOLTAGE_DIVISOR

#ifdef HAVE_GPS //no multipage without GPS
      varioScreen.nextPage();
#endif //HAVE_GPS multipage support      
    }
  }

   
  /*****************/
  /* update screen */
  /*****************/
#ifdef HAVE_GPS
  /* when getting speed from gps, display speed and ratio */
  if ( nmeaParser.haveNewSpeedValue() ) {

    double currentSpeed = nmeaParser.getSpeed();
    double ratio = history.getGlideRatio(currentSpeed, serialNmea.getReceiveTimestamp());

    speedDigit.setValue( currentSpeed );
    if( currentSpeed >= RATIO_MIN_SPEED && ratio >= 0.0 && ratio < RATIO_MAX_VALUE ) {
      ratioDigit.setValue(ratio);
    } else {
      ratioDigit.setValue(0.0);
    }
  }
#endif //HAVE_GPS

  /* screen update */
  varioScreen.displayStep();

#endif //HAVE_SCREEN 

#ifdef HAVE_LOW_BATTERY_BEEP
  makeBatteryBeeps();
#endif
 
}

#ifdef HAVE_LOW_BATTERY_BEEP
void makeBatteryBeeps(){
  if ( millis() - batteryBeeps_lastrun > BATTERY_BEEPS_INTERVAL ){
    batteryBeeps_lastrun = millis(); 
    #ifdef HAVE_VOLTAGE_DIVISOR
    // To be implemented
    #endif // HAVE_VOLTAGE_DIVISOR
    #ifdef HAVE_INTERNAL_VOLTMETER
    if (readVcc() < LOW_BATT_BEEP_THRESOLD)
    {
      for (char i = 0; i < 5; i++)
      {
        toneAC(1000,10);
        delay(50);
        toneAC(0,0);
        delay(50);
      }
      
    }
    #endif //HAVE_INTERNAL_VOLTMETER

  }
}

long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}
#endif // HAVE_LOW_BATTERY_BEEP

#if defined(HAVE_SDCARD) && defined(HAVE_GPS)
void createSDCardTrackFile(void) {
  /* start the sdcard record */
  if( sdcardState == SDCARD_STATE_INITIALIZED ) {

    /* some cards doesn't like delays between init and write, so reinit */
    file.init();

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
    if( file.begin((char*)dateChar, 8) >= 0 ) {
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
  /* set base time */
#if defined(HAVE_SCREEN) && defined(HAVE_GPS)
  screenElapsedTime.setBaseTime( screenTime.getTime() );
#endif //defined(HAVE_SCREEN) && defined(HAVE_GPS)

  /* enable near climbing */
#ifdef HAVE_SPEAKER
#ifdef VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM
  beeper.setGlidingAlarmState(true);
#endif
#ifdef VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP
  beeper.setGlidingBeepState(true);
#endif
#endif //HAVE_SPEAKER

#if defined(HAVE_SDCARD) && defined(HAVE_GPS) && defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)
  createSDCardTrackFile();
#endif // defined(HAVE_SDCARD) && defined(VARIOMETER_RECORD_WHEN_FLIGHT_START)
}
