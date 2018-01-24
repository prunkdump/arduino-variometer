#include <Arduino.h>
#include <VarioSettings.h>
#include <I2Cdev.h>
#include <ms5611.h>
#include <vertaccel.h>
#include <EEPROM.h>
#include <LightInvensense.h>
#include <avr/pgmspace.h>
#include <FirmwareUpdater.h>

#define CALIBRATING_DISPLAY_DELAY 3000
#define RESULT_DISPLAY_DELAY 1000

double magCalibration[3];
short magMinValues[3];
short magMaxValues[3];
bool extUpdated[3];

unsigned long lastDisplayTimestamp;

#define STATE_INITIAL 0
#define STATE_CALIBRATING 1
#define STATE_DISPLAY 2
#define STATE_CALIBRATED 3
int state = STATE_INITIAL;

void setup() {
  delay(2000);

  /* init serial */
  Serial.begin(9600);

  /* init device */
  Fastwire::setup(FASTWIRE_SPEED, 0);
  vertaccel_init();

  /* read current calibration */
  vertaccel_getMagCalibration(magCalibration);
  Serial.print("------------------\n");
  Serial.print("Current calibration coefficients :\n");
  for( uint8_t i = 0; i<3; i++ ) {
    Serial.println(magCalibration[i], 3);
  }
  Serial.print("------------------\n");
  Serial.print("Press 'c' to start or reset calibration.\n");
  Serial.print("Press 's' to save calibration.\n");
  Serial.print("Press 'd' to display calibrated mag.\n");

  /* init var */
  while( ! fastMPUMagReady() ) ;
  fastMPUReadMag( magMinValues );
  for( int i = 0; i<3; i++) {
    magMaxValues[i] = magMinValues[i];
  }

}

short newMag[3];

void loop() {

  /*************************/
  /* update mag ext values */
  /*************************/
  if( fastMPUMagReady() ) {
    
    fastMPUReadMag( newMag );

    if( state == STATE_CALIBRATING ) {

      for( int i = 0; i<3; i++) {
        if( newMag[i] < magMinValues[i] ) {
          magMinValues[i] = newMag[i];
          extUpdated[i] = true;
        }
        if( newMag[i] > magMaxValues[i] ) {
          magMaxValues[i] = newMag[i];
          extUpdated[i] = true;
        }
      }
    }
  }

  /******************/
  /* check commands */
  /******************/
  if( Serial.available() ) {
    uint8_t c = Serial.read();

    /* calibrate */
    if( c == 'c' ) {
      Serial.print("------------------\n");
    
      if( state != STATE_DISPLAY ) {
        Serial.print("Starting calibration.\n");

        for( int i = 0; i<3; i++) {
          magCalibration[i] = 0;
          extUpdated[i] = false;
        }

        while( ! fastMPUMagReady() ) ;
        fastMPUReadMag( magMinValues );
        for( int i = 0; i<3; i++) {
          magMaxValues[i] = magMinValues[i];
        }
      }

      else {
        Serial.print("Return to calibration.\n");
      }
      
      lastDisplayTimestamp = millis();
      state = STATE_CALIBRATING;
    }

    /* save or display */
    if( c == 's' || c == 'd' ) {
      Serial.print("------------------\n");
      if( c == 's' ) {
        Serial.print("Saved calibration coefficients :\n");
      } else {
        Serial.print("Current calibration coefficients :\n");
      }

      if( state == STATE_CALIBRATING ) {
        for( int i = 0; i<3; i++) {
          magCalibration[i] = -((double)magMinValues[i] + (double)magMaxValues[i])/2.0;
          Serial.println(magCalibration[i], 3);
        }
      }

      if( c == 's' ) {
        vertaccel_saveMagCalibration(magCalibration);
        state = STATE_CALIBRATED;
      } else {
        state = STATE_DISPLAY;
      }
      lastDisplayTimestamp = millis();
    }
  }
      
      

  /***********/
  /* display */
  /***********/

  /* calibrating case */
  if( state == STATE_CALIBRATING ) {
    
    if( millis() - lastDisplayTimestamp > CALIBRATING_DISPLAY_DELAY ) {

      lastDisplayTimestamp = millis();
      Serial.print("------------------\n");
    
      /* update mesg */
      Serial.print("Updated = (");
      for( int i = 0; i<3; i++ ) {
        if( extUpdated[i] ) {
          Serial.print(" * ");
        } else {
          Serial.print("   ");
        }
        extUpdated[i] = false;
        if( i != 2 ) {
          Serial.print(",");
        } else {
          Serial.print(")\n");
        }
      }
    
      /* display values */
      Serial.print("MIN = (");
      Serial.print(magMinValues[0], DEC);
      Serial.print(", ");
      Serial.print(magMinValues[1], DEC);
      Serial.print(", ");
      Serial.print(magMinValues[2], DEC);
      Serial.print(")\n");

      Serial.print("MAX = (");
      Serial.print(magMaxValues[0], DEC);
      Serial.print(", ");
      Serial.print(magMaxValues[1], DEC);
      Serial.print(", ");
      Serial.print(magMaxValues[2], DEC);
      Serial.print(")\n");

      Serial.print("LEN = (");
      Serial.print(magMaxValues[0] - magMinValues[0], DEC);
      Serial.print(", ");
      Serial.print(magMaxValues[1] - magMinValues[1], DEC);
      Serial.print(", ");
      Serial.print(magMaxValues[2] - magMinValues[2], DEC);
      Serial.print(")\n");
    }
  }

  /* display or calibratted  */
  else {
    if( state != STATE_INITIAL ) {
      if( millis() - lastDisplayTimestamp > RESULT_DISPLAY_DELAY ) {

        lastDisplayTimestamp = millis();
        Serial.print("------------------\n");

        Serial.print( "(" );
        Serial.print(newMag[0] + (short)magCalibration[0], DEC);
        Serial.print( ", " );
        Serial.print(newMag[1] + (short)magCalibration[1], DEC);
        Serial.print( ", " );
        Serial.print(newMag[2] + (short)magCalibration[2], DEC);
        Serial.print( ")\n" );
     }
    }
  }
}
  

