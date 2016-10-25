#include <Arduino.h>
#include <I2Cdev.h>
#include <ms5611.h>
#include <vertaccel.h>
#include <inv_mpu.h>
#include <kalmanvert.h>
#include <beeper.h>
#include <toneAC.h>
#include <SPI.h>
#include <avr/pgmspace.h>
#include <varioscreen.h>
#include <SdCard.h>
#include <LightFat16.h>
#include <lightnmea.h>


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
#define VARIOSCREEN_SPEED SPI_CLOCK_DIV4
#define SDCARD_SPEED SPI_CLOCK_DIV4

/*!!!!!!!!!!!!!!!!!!!!!!!*/
/* VARIOMETER PARAMETERS */
/*!!!!!!!!!!!!!!!!!!!!!!!*/
#define VARIOMETER_SINKING_THRESHOLD -2.0
#define VARIOMETER_CLIMBING_THRESHOLD 0.2
#define VARIOMETER_NEAR_CLIMBING_SENSITIVITY 0.5

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
#endif

/***************/
/* gps objects */
/***************/
#ifdef HAVE_GPS

#define GPS_CALIBRATION_STEPS 5

NmeaParser parser;
boolean gpsDataStarted = false;
boolean gpsAltiCalibrated = false;
unsigned char gpsAltiCalibrationStep = 0;
unsigned long lastSpeedTimestamp;
double lastAltiValue;

#ifdef HAVE_SDCARD
lightfat16 file;
boolean sdcardFound;
#endif //HAVE_SDCARD

#endif //HAVE_GPS

/*-----------------*/
/*      SETUP      */
/*-----------------*/
void setup() {
  beeper.setGlidingBeepState(true);
  beeper.setGlidingAlarmState(true);
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
   
  /************************************/
  /* init altimeter and accelerometer */
  /************************************/
  Fastwire::setup(400,0);
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
   
#ifdef HAVE_GPS               
  /* init screen gps vars */
  lastSpeedTimestamp = millis();
  lastAltiValue = ms5611_getAltitude();
#endif //HAVE_GPS  
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
  }

  /*****************/
  /* update beeper */
  /*****************/
#ifdef HAVE_SPEAKER
  beeper.update();
#endif //HAVE_SPEAKER

  /**************/
  /* update gps */
  /**************/
#ifdef HAVE_GPS
  if( Serial.available() > 0 ) {
    
    /* if needed wait for GPS initialization */
    if( ! gpsDataStarted ) {
      while (Serial.available() > 0) {
        int c = Serial.read();
        if( c == '$' ) {
          parser.getChar(c);
#ifdef HAVE_SDCARD
          if( sdcardFound ) {
            file.write(c);
          }
#endif //HAVE_SDCARD
          gpsDataStarted = true;
        }
      }
    }
    
    /* else parse NMEA and save to sdcard */
    if( gpsDataStarted ) {
#ifdef HAVE_SDCARD
      boolean altiSaved = false;
#endif //HAVE_SDCARD
      while (Serial.available() > 0) {
        int c = Serial.read();
        parser.getChar(c);
#ifdef HAVE_SDCARD
        if( sdcardFound ) {
          file.write(c);
          /*--------------------------------*/
          /* save barometric alti if needed */
          /*--------------------------------*/
          if( ! altiSaved && parser.haveNewSpeedValue() && gpsAltiCalibrated ) { //just after $GPRMC
            file.write('\r');
            file.write('\n'); // the newline of the $GPRMC line will be writed after
            uint8_t altiDigits[13];
            unsigned currentAlti = (kalmanvert.getPosition()*10.0);
            uint8_t parity = (((('$'^'B')^'A')^',')^'.'); //one digit after .
            altiDigits[3] = '0'+ currentAlti%10;
            currentAlti /= 10;
            altiDigits[4] = '.';
            parity ^= altiDigits[3];
            
            int8_t pos = 5;
            while( currentAlti != 0) {
              altiDigits[pos] =  '0' + currentAlti%10;
              currentAlti /= 10;
              pos++;
            }
            
            /* constants */
            altiDigits[pos] = ',';
            altiDigits[pos+1] = 'A';
            altiDigits[pos+2] = 'B';
            altiDigits[pos+3] = '$';
            pos+=3;
  
            /* parity */
            altiDigits[0] = '0' + parity%10;
            altiDigits[1] = '0' + parity/10;
            altiDigits[2] = '*';
            
            while(pos >= 0) {
              file.write(altiDigits[pos]);
              pos--;
            }
            
            altiSaved = true;
          }
        }
#endif //HAVE_SDCARD
      }
    }
  }
  
  /* recalibrate alti with gps */
  if( ! gpsAltiCalibrated ) {
   if( parser.haveNewAltiValue() ) {
     gpsAltiCalibrationStep++;
     if( gpsAltiCalibrationStep == GPS_CALIBRATION_STEPS) { //get 5 alti values before calibrating
       double gpsAlti = parser.getAlti();
       ms5611_setCurrentAltitude(gpsAlti);
       kalmanvert.resetPosition(gpsAlti);
       gpsAltiCalibrated = true;
     }
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
  if ( parser.haveNewSpeedValue() ) {
    double currentSpeed = parser.getSpeed();
       
    /* compute ratio */
    unsigned long duration = millis() - lastSpeedTimestamp;
    lastSpeedTimestamp = millis();
    double altiDelta = lastAltiValue- kalmanvert.getPosition(); //give positive result 
    lastAltiValue = kalmanvert.getPosition();
    double ratio = (currentSpeed*((double)duration)/3600.0)/altiDelta;
    
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
