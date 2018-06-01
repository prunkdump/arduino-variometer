#include <Arduino.h>
#include <VarioSettings.h>
#include <I2Cdev.h>
#include <ms5611.h>
#include <vertaccel.h>
#include <EEPROM.h>
#include <LightInvensense.h>
#include <avr/pgmspace.h>
#include <toneAC.h>
#include <FirmwareUpdater.h>

#define VOLUME 3
#define SAVE_WHEN_NO_UPDATE_DELAY 15000
#define MEASURE_DELAY 500
#define BEEP_DURATION 200
#define BEEP_FREQ 1000
#define BEEP_END_FREQ 100

bool extUpdated;
unsigned long lastMeasureTimestamp;
unsigned long lastUpdateTimestamp;

double magCalibration[3];
int16_t magMinValues[3];
int16_t magMaxValues[3];


#define STATE_INITIAL 0
#define STATE_CALIBRATED 1 
int state = STATE_INITIAL;

Vertaccel vertaccel;

void setup() {

  /* start devices */
  delay(VARIOMETER_POWER_ON_DELAY);
  Fastwire::setup(FASTWIRE_SPEED, 0);
  vertaccel.init();
  if( firmwareUpdateCond() ) {
   firmwareUpdate();
  }
  
  /* set vars */
  while( ! fastMPUMagReady() ) ;
  fastMPUReadMag( magMinValues );
  for( int i = 0; i<3; i++) {
    magMaxValues[i] = magMinValues[i];
  }

  for( int i = 0; i<3; i++) {
    magCalibration[i] = 0.0;
  }
 
  lastMeasureTimestamp =millis();
  lastUpdateTimestamp = millis();

}

int16_t newMag[3];

void loop() {

  if( state == STATE_INITIAL ) {

    /*************************/
    /* update mag ext values */
    /*************************/
    if( fastMPUMagReady() ) {
    
      fastMPUReadMag( newMag );
      for( int i = 0; i<3; i++) {
        if( newMag[i] < magMinValues[i] ) {
          magMinValues[i] = newMag[i];
          extUpdated = true;
          lastUpdateTimestamp = millis();
        }
        if( newMag[i] > magMaxValues[i] ) {
          magMaxValues[i] = newMag[i];
          extUpdated = true;
          lastUpdateTimestamp = millis();
        }
      }
    }

    /*****************/
    /* check signals */
    /*****************/
    if( millis() - lastMeasureTimestamp > MEASURE_DELAY ) {

      lastMeasureTimestamp = millis();

      /* need beep ? */
      if( extUpdated ) {
        toneAC(BEEP_FREQ, VOLUME);
      }
      extUpdated = false;
    }

    else if( millis() - lastMeasureTimestamp > BEEP_DURATION ) {
      toneAC(0);
    }


    /**************/
    /* check done */
    /**************/
    if( millis() - lastUpdateTimestamp > SAVE_WHEN_NO_UPDATE_DELAY ) {
      toneAC(BEEP_END_FREQ, VOLUME);
      
      for( int i = 0; i<3; i++) {
        magCalibration[i] = ((double)magMinValues[i] + (double)magMaxValues[i])/2.0;
      }
      VertaccelCalibration magCal = {{ (int16_t)(magCalibration[0]*(double)(1 << VERTACCEL_MAG_CAL_BIAS_MULTIPLIER))
                                         ,(int16_t)(magCalibration[1]*(double)(1 << VERTACCEL_MAG_CAL_BIAS_MULTIPLIER))
                                         ,(int16_t)(magCalibration[2]*(double)(1 << VERTACCEL_MAG_CAL_BIAS_MULTIPLIER)) }, VERTACCEL_DEFAULT_MAG_CAL_PROJ_SCALE};
      vertaccel.saveMagCalibration(magCal);
      
      delay(2000);
      toneAC(0); 
      state = STATE_CALIBRATED;
    }
  }
}
   
