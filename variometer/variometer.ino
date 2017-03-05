#include <Arduino.h>
#include <I2Cdev.h>
#include <ms5611.h>
#include <vertaccel.h>
#include <EEPROM.h>
#include <inv_mpu.h>
#include <kalmanvert.h>
#include <beeper.h>
#include <toneAC.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include <varioscreen.h>
#include <digit.h>
#include <SdCard.h>
#include <LightFat16.h>
#include <nmea.h>


/*!!!!!!!!!!!!!!!!!!!!!!!*/
/* VARIOMETER STRUCTURE  */
/*!!!!!!!!!!!!!!!!!!!!!!!*/
#define HAVE_SPEAKER
#define HAVE_ACCELEROMETER
#define HAVE_SCREEN
#define HAVE_GPS
#define HAVE_SDCARD

#define VARIOSCREEN_DC_PIN 6
#define VARIOSCREEN_CS_PIN 2
#define VARIOSCREEN_RST_PIN 4
#define SDCARD_CS_PIN 8

// the variometer seems to be more stable at half speed
// don't hesitate to experiment
#if F_CPU >= 16000000L
#define VARIOSCREEN_SPEED SPI_CLOCK_DIV4
#define SDCARD_SPEED SPI_CLOCK_DIV4
#else
#define VARIOSCREEN_SPEED SPI_CLOCK_DIV2
#define SDCARD_SPEED SPI_CLOCK_DIV2
#endif //CPU_FREQ

/*!!!!!!!!!!!!!!!!!!!!!!!*/
/* VARIOMETER PARAMETERS */
/*!!!!!!!!!!!!!!!!!!!!!!!*/
#define VARIOMETER_SINKING_THRESHOLD -2.0
#define VARIOMETER_CLIMBING_THRESHOLD 0.2
#define VARIOMETER_NEAR_CLIMBING_SENSITIVITY 0.5

#define VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM
#define VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP

/* mean filter duration = filter size * 2 seconds */
#define VARIOMETER_SPEED_FILTER_SIZE 5

/*****************/
/* screen objets */
/*****************/
#ifdef HAVE_SCREEN
#ifdef HAVE_GPS 
#define VARIOSCREEN_ALTI_ANCHOR_X 58
#define VARIOSCREEN_ALTI_ANCHOR_Y 0
#define VARIOSCREEN_VARIO_ANCHOR_X 58
#define VARIOSCREEN_VARIO_ANCHOR_Y 2
#define VARIOSCREEN_SPEED_ANCHOR_X 22
#define VARIOSCREEN_SPEED_ANCHOR_Y 4
#define VARIOSCREEN_GR_ANCHOR_X 72
#define VARIOSCREEN_GR_ANCHOR_Y 4
#define RATIO_MAX_VALUE 30.0
#define RATIO_MIN_SPEED 10.0
#else
#define VARIOSCREEN_ALTI_ANCHOR_X 58
#define VARIOSCREEN_ALTI_ANCHOR_Y 1
#define VARIOSCREEN_VARIO_ANCHOR_X 58
#define VARIOSCREEN_VARIO_ANCHOR_Y 3
#endif //HAVE_GPS

VarioScreen screen(VARIOSCREEN_DC_PIN,VARIOSCREEN_CS_PIN,VARIOSCREEN_RST_PIN);
MSUnit msunit(screen);
MUnit munit(screen);
ScreenDigit altiDigit(screen, VARIOSCREEN_ALTI_ANCHOR_X, VARIOSCREEN_ALTI_ANCHOR_Y, 0, false);
ScreenDigit varioDigit(screen, VARIOSCREEN_VARIO_ANCHOR_X, VARIOSCREEN_VARIO_ANCHOR_Y, 1, true);
#ifdef HAVE_GPS 
ScreenDigit speedDigit(screen, VARIOSCREEN_SPEED_ANCHOR_X, VARIOSCREEN_SPEED_ANCHOR_Y, 0, false);
ScreenDigit ratioDigit(screen, VARIOSCREEN_GR_ANCHOR_X, VARIOSCREEN_GR_ANCHOR_Y, 1, false);
KMHUnit kmhunit(screen);
GRUnit grunit(screen);
#endif //HAVE_GPS
unsigned char screenStatus;
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
beeper beeper(VARIOMETER_SINKING_THRESHOLD, VARIOMETER_CLIMBING_THRESHOLD, VARIOMETER_NEAR_CLIMBING_SENSITIVITY);

#define FLIGHT_START_MIN_TIMESTAMP 15000
#define FLIGHT_START_VARIO_LOW_THRESHOLD (-0.5)
#define FLIGHT_START_VARIO_HIGH_THRESHOLD 0.5
#define FLIGHT_START_MIN_SPEED 10.0
boolean beepNearThermalEnabled = false;

#endif

/***************/
/* gps objects */
/***************/
#ifdef HAVE_GPS

Nmea nmea;
boolean gpsAltiCalibrated = false;

unsigned long speedFilterTimestamps[VARIOMETER_SPEED_FILTER_SIZE];
double speedFilterSpeedValues[VARIOMETER_SPEED_FILTER_SIZE];
double speedFilterAltiValues[VARIOMETER_SPEED_FILTER_SIZE];
int8_t speedFilterPos = 0;

#ifdef HAVE_SDCARD
lightfat16 file;
boolean sdcardFound;
#endif //HAVE_SDCARD

#endif //HAVE_GPS

/*-----------------*/
/*      SETUP      */
/*-----------------*/
void setup() {
  /****************/
  /* init sdcard  */
  /****************/
#ifdef HAVE_GPS
#ifdef HAVE_SDCARD
  if( file.init(SDCARD_CS_PIN, SDCARD_SPEED) >= 0 ) {
    sdcardFound = true;
   } else {
    sdcardFound = false;
   }
#endif //HAVE_SDCARD
#endif //HAVE_GPS

  /***************/
  /* init screen */
  /***************/
#ifdef HAVE_SCREEN
  screen.begin(VARIOSCREEN_SPEED);
  munit.display(VARIOSCREEN_ALTI_ANCHOR_X, VARIOSCREEN_ALTI_ANCHOR_Y);
  msunit.display(VARIOSCREEN_VARIO_ANCHOR_X, VARIOSCREEN_VARIO_ANCHOR_Y);
#ifdef HAVE_GPS
  kmhunit.display(VARIOSCREEN_SPEED_ANCHOR_X, VARIOSCREEN_SPEED_ANCHOR_Y);
  grunit.display(VARIOSCREEN_GR_ANCHOR_X, VARIOSCREEN_GR_ANCHOR_Y);
#endif //HAVE_GPS
#endif //HAVE_SCREEN
  
  /************************************/
  /* init altimeter and accelerometer */
  /************************************/
  /* !!! fastwire don't take on account the cpu frequency !!! */
#if F_CPU >= 16000000L
  Fastwire::setup(400,0);
#else
  Fastwire::setup(800,0);
#endif //CPU_FREQ
  ms5611_init();
#ifdef HAVE_ACCELEROMETER
  vertaccel_init();
#endif //HAVE_ACCELEROMETER
  
  /************/
  /* init gps */
  /************/
#ifdef HAVE_GPS
  Serial.begin(9600);
#endif //HAVE_GPS
  
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
   
}

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

    /* set nmea data */
#ifdef HAVE_GPS
    nmea.setBaroData( kalmanvert.getPosition(), kalmanvert.getVelocity() );
#endif //HAVE_GPS
  }

  /*****************/
  /* update beeper */
  /*****************/
#ifdef HAVE_SPEAKER
  beeper.update();

  /* check if near thermal features need to be started */
#if defined(VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM) || defined(VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP)
  if( !beepNearThermalEnabled ) {
    /* check flight start conditions */
    if( (millis() > FLIGHT_START_MIN_TIMESTAMP) &&
        (kalmanvert.getVelocity() < FLIGHT_START_VARIO_LOW_THRESHOLD || kalmanvert.getVelocity() > FLIGHT_START_VARIO_HIGH_THRESHOLD)
#ifdef HAVE_GPS
        && gpsAltiCalibrated && (nmea.getSpeed() > FLIGHT_START_MIN_SPEED)
#endif //HAVE_GPS
      ) {
    beepNearThermalEnabled = true;
#ifdef VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM
    beeper.setGlidingAlarmState(true);
#endif
#ifdef VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP
    beeper.setGlidingBeepState(true);
#endif
    }
  }  
#endif //defined(VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM) || defined(VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP)
#endif //HAVE_SPEAKER

  /**************/
  /* update gps */
  /**************/
#ifdef HAVE_GPS
  if( Serial.available() > 0 ) {
    
    /* feed nmea */
    nmea.feed(Serial.read());
    
    /* check if we need to calibrate */
    if( ! gpsAltiCalibrated ) {
      if( nmea.ready() ) {
        double gpsAlti = nmea.getAlti();
        ms5611_setCurrentAltitude(gpsAlti);
        kalmanvert.resetPosition(gpsAlti);
        gpsAltiCalibrated = true;
        nmea.setBaroData(gpsAlti, kalmanvert.getVelocity());
      }
    }
    
    /* read nmea and output if needed */
    while( nmea.availiable() ) {
      uint8_t oc = nmea.read();
#ifdef HAVE_SDCARD
          if( sdcardFound ) {
            file.write(oc);
          }
#endif //HAVE_SDCARD
    }
  }
#endif //HAVE_GPS
  
  /*****************/
  /* update screen */
  /*****************/
#ifdef HAVE_SCREEN
  /* alternate display : alti / vario */
  if( screenStatus == 0 ) {
    altiDigit.display( kalmanvert.getPosition() );
    screenStatus = 1;
  } else {
    varioDigit.display( kalmanvert.getVelocity() );
    screenStatus = 0;
  }
  
#ifdef HAVE_GPS
  /* when getting speed from gps, display speed and ratio */
  if ( nmea.haveNewSpeedValue() ) {

    /* get new values */
    unsigned long baseTime = speedFilterTimestamps[speedFilterPos];
    unsigned long deltaTime = millis(); //computed later
    speedFilterTimestamps[speedFilterPos] = deltaTime;
    
    double deltaAlti = speedFilterAltiValues[speedFilterPos]; //computed later
    speedFilterAltiValues[speedFilterPos] = kalmanvert.getPosition(); 

    double currentSpeed = nmea.getSpeed();
    speedFilterSpeedValues[speedFilterPos] = currentSpeed;

    speedFilterPos++;
    if( speedFilterPos >= VARIOMETER_SPEED_FILTER_SIZE )
      speedFilterPos = 0;

    /* compute deltas */
    deltaAlti -= kalmanvert.getPosition();
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
    speedDigit.display( currentSpeed );
    if( currentSpeed >= RATIO_MIN_SPEED && ratio >= 0.0 && ratio < RATIO_MAX_VALUE ) {
      ratioDigit.display(ratio);
    } else {
      ratioDigit.display(0.0);
    }
  }
#endif //HAVE_GPS
#endif //HAVE_SCREEN 
}
