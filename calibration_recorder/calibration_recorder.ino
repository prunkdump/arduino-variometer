/* calibration_recorder -- Record accel and mag
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
#include <EEPROM.h>
#include <LightInvensense.h>
#include <toneAC.h>
#include <avr/pgmspace.h>
#include <digit.h>
#include <SdCard.h>
#include <LightFat16.h>
#include <FirmwareUpdater.h>

/* where output ? */
//#define SERIAL_OUTPUT
#define SDCARD_OUTPUT

/* need beeps */
#define MAKE_BEEP
#define BEEP_DURATION 300
#define BEEP_VOLUME 3
#define BEEP_START_FREQ 500
#define BEEP_RECORD_FREQ 1000

/* to compute the standard deviation of the accelerometer */
#define ACCEL_SD_WAIT_DURATION 2000
#define ACCEL_SD_MEASURE_DURATION 8000

/* to make averaged measure */
#define MEASURE_DURATION 500
#define STABILIZE_DURATION 500

/* movement detection */
#define PREDICTION_INTERVAL_COEFFICIENT 1.96

/* orientation change detection */
#define NEW_MEASURE_MINIMAL_DEVIATION_COEFF 20.0



/* global variables */

#ifdef SDCARD_OUTPUT
lightfat16 file(SDCARD_CS_PIN);
boolean sdcardFound = false;
char filename[] = "RECORD00";
#define FILENAME_SIZE 8
Digit valueDigit;

void writeNumber(int16_t number) {
  valueDigit.begin((int32_t)number);
  while( valueDigit.available() ) {
    file.write( valueDigit.get() );
  }
}
#endif //SDCARD_OUTPUT


/********************************/
/* Standard deviation recording */
/********************************/
unsigned long accelSDRecordTimestamp;
double rawAccelSD;

#define RECORD_STATE_INITIAL 0
#define RECORD_STATE_WAIT_DONE 1
#define RECORD_STATE_ACCEL_SD_RECORDED 2
#define RECORD_STATE_GYRO_CALIBRATED 3
int recordInitState = RECORD_STATE_INITIAL;


/*****************/
/* measures data */
/*****************/
unsigned long measureTimestamp;

/* accel measures */
int16_t lastAccelMeasure[3];
long accelCount;
double accelMean[3];
double accelSD[3];

/* mag measures */
int16_t lastMagMeasure[3];
long magCount;
double magMean[3];
double magSD[3];


void startMeasure(void) {

  /* stabilize */
  unsigned long stabilizeTimestamp = millis();
  while( millis() - stabilizeTimestamp < STABILIZE_DURATION ) {
    makeMeasureStep();
  }

  /* init vars */
  measureTimestamp = millis();
  accelCount = 0;
  magCount = 0;

  for( int i = 0; i<3; i++) {
    accelMean[i] = 0.0;
    accelSD[i] = 0.0;
    magMean[i] = 0.0;
    magSD[i] = 0.0;
  }
}

uint8_t readRawAccel(int16_t* accel, int32_t* quat) {

  uint8_t haveValue = 0;

  while( fastMPUReadFIFO(NULL, accel, quat) >= 0 ) {
    haveValue = 1;
  }

  return haveValue;
}

#ifdef AK89xx_SECONDARY
uint8_t readRawMag(int16_t* mag) {

  uint8_t haveValue = 0;

  if( fastMPUMagReady() ) {
#ifdef VERTACCEL_USE_MAG_SENS_ADJ
    fastMPUReadMag(mag);
#else
    fastMPUReadRawMag(mag);
#endif //VERTACCEL_USE_MAG_SENS_ADJ
    haveValue = 1;
  }

  return haveValue;
}
#endif //AK89xx_SECONDARY


void makeMeasureStep(void) {

  /* accel */
  int16_t accel[3];
  int32_t quat[4];
  if( readRawAccel(accel, quat) ) {
    accelCount++;
    for( int i = 0; i<3; i++) {
      accelMean[i] += (double)accel[i];
      accelSD[i] += ((double)accel[i])*((double)accel[i]);
    }
  }

  /* mag */
  int16_t mag[3];
  if( readRawMag(mag) ) {
    magCount++;
    for( int i = 0; i<3; i++) {
      magMean[i] += (double)mag[i];
      magSD[i] += ((double)mag[i])*((double)mag[i]);
    }
  }
}


/* return standard deviation */
double getAccelMeasure(int16_t* accelMeasure) {
  
  double accelMeasureSD = 0.0; 
    
  for( int i = 0; i<3; i++) {
    accelMeasureSD += accelSD[i]/(double)accelCount;
    accelMeasureSD -= (accelMean[i]/(double)accelCount) * (accelMean[i]/(double)accelCount);
    
    accelMeasure[i] = (int16_t)(accelMean[i]/(double)accelCount);
  }

  return sqrt(accelMeasureSD);
}


/* return standard deviation */
double getMagMeasure(int16_t* magMeasure) {
  
  double magMeasureSD = 0.0; 
    
  for( int i = 0; i<3; i++) {
    magMeasureSD += magSD[i]/(double)magCount;
    magMeasureSD -= (magMean[i]/(double)magCount) * (magMean[i]/(double)magCount);
    
    magMeasure[i] = (int16_t)(magMean[i]/(double)magCount);
  }

  return sqrt(magMeasureSD);
}



void setup() {

#ifdef SERIAL_OUTPUT
  /* init serial */
  Serial.begin(9600);
#endif //SERIAL_OUTPUT

  /* start devices */
  delay(VARIOMETER_POWER_ON_DELAY);
  intTW.begin();
  fastMPUInit(true);
  if( firmwareUpdateCond() ) {
   firmwareUpdate();
  }

#ifdef SDCARD_OUTPUT
  /* init sd card */
  if( file.init() >= 0 ) {
    sdcardFound = true;
  }
#endif //SDCARD_OUTPUT

  /* reset variables */
  startMeasure();
  accelSDRecordTimestamp = millis(); 
  
}

void loop() {

  /*------------------------------------------------------------*/
  /*  wait, record standard deviation, and get gyro calibration */
  /*------------------------------------------------------------*/

  /* first  wait */
  if( recordInitState == RECORD_STATE_INITIAL ) {

    /* wait finished ? */
    if( millis() - accelSDRecordTimestamp > ACCEL_SD_WAIT_DURATION ) {

      /* start measure */
#ifdef MAKE_BEEP
      toneAC(BEEP_START_FREQ, BEEP_VOLUME);
      delay(BEEP_DURATION);
      toneAC(0);
#endif //MAKE_BEEP     
      startMeasure();

      /* next step */
      recordInitState = RECORD_STATE_WAIT_DONE;
    }
  }

  /* next record accel SD */
  else if( recordInitState == RECORD_STATE_WAIT_DONE ) {

    /* accel SD recording finished ? */
    if( millis() - measureTimestamp > ACCEL_SD_MEASURE_DURATION ) {

      /* get result */
      int16_t measure[3];
      rawAccelSD = getAccelMeasure(measure);

      /* next step */
      recordInitState = RECORD_STATE_ACCEL_SD_RECORDED;
    }

    /* else measure */
    else {
      makeMeasureStep();
    }
  }

  /* next get gyro calibration */
  else if( recordInitState == RECORD_STATE_ACCEL_SD_RECORDED ) {

    /* check if gyro calibrated */
    unsigned char gyroCal[12];
    fastMPUReadGyroBias(gyroCal);

    bool gyroCalibrated = false;
    for(int i = 0; i<12; i++) {
      if(gyroCal[i] != 0 ) {
        gyroCalibrated = true;
        break;
      }
    }

    if( ! gyroCalibrated ) {
      delay(MEASURE_DURATION);
    }

    /* output gyro calibration start measures */
    else {
      
#ifdef SDCARD_OUTPUT
      if( sdcardFound ) {
        file.begin(filename, FILENAME_SIZE);
      }

      writeNumber(gyroCal[0]);
      for(int i = 1; i<12; i++) {
        file.write(',');
        file.write(' ');
        writeNumber(gyroCal[i]);
      }
      file.write('\n');
#endif //SDCARD_OUTPUT

#ifdef SERIAL_OUTPUT
      Serial.print(gyroCal[0], DEC);
      for(int i = 1; i<12; i++) {
        Serial.print(", ");
        Serial.print(gyroCal[i], DEC);
      }
      Serial.print("\n");
#endif //SERIAL_OUTPUT

      /* start recording */
      recordInitState = RECORD_STATE_GYRO_CALIBRATED;
      startMeasure();
    }
  }

  /*-----------------*/
  /* record measures */
  /*-----------------*/
  else {

    /****************/
    /* make measure */
    /****************/
    if( millis() - measureTimestamp < MEASURE_DURATION ) {
      makeMeasureStep();
    }

    /******************/
    /* output measure */
    /******************/
    else {

      int16_t accelMeasure[3];
      double accelMeasureSD = getAccelMeasure(accelMeasure);

      int16_t magMeasure[3];
      double magMeasureSD = getMagMeasure(magMeasure);

    
      /**************************/
      /* check measure validity */
      /**************************/

      /* check measure stability */
      if( accelMeasureSD < PREDICTION_INTERVAL_COEFFICIENT * rawAccelSD ) {

        /* check deviation with last measure */
        double measureDeviation = 0.0;
        for( int i = 0; i<3; i++) {
          double s = (double)accelMeasure[i] - (double)lastAccelMeasure[i];
          measureDeviation += s * s;
        }
        measureDeviation = sqrt(measureDeviation);

        if( measureDeviation > NEW_MEASURE_MINIMAL_DEVIATION_COEFF*PREDICTION_INTERVAL_COEFFICIENT*rawAccelSD/sqrt((double)accelCount) ) {

          /* save measure */
          for( int i = 0; i<3; i++) {
            lastAccelMeasure[i] = accelMeasure[i];
          }


#ifdef SERIAL_OUTPUT
          /*****************/
          /* serial output */
          /*****************/
          Serial.print(accelMeasure[0], DEC);
          Serial.print(", ");
          Serial.print(accelMeasure[1], DEC);
          Serial.print(", ");
          Serial.print(accelMeasure[2], DEC);
          Serial.print(", ");
          Serial.print(magMeasure[0], DEC);
          Serial.print(", ");
          Serial.print(magMeasure[1], DEC);
          Serial.print(", ");
          Serial.print(magMeasure[2], DEC);
          Serial.print("\n");
#endif //SERIAL_OUTPUT

#ifdef SDCARD_OUTPUT
          /*****************/
          /* SDcard output */
          /*****************/
          writeNumber(accelMeasure[0]);
          file.write(',');
          file.write(' ');
          writeNumber(accelMeasure[1]);
          file.write(',');
          file.write(' ');
          writeNumber(accelMeasure[2]);
          file.write(',');
          file.write(' ');
          writeNumber(magMeasure[0]);
          file.write(',');
          file.write(' ');
          writeNumber(magMeasure[1]);
          file.write(',');
          file.write(' ');
          writeNumber(magMeasure[2]);
          file.write('\n');
          
          file.sync();
#endif //SDCARD_OUTPUT

#ifdef MAKE_BEEP
          toneAC(BEEP_RECORD_FREQ, BEEP_VOLUME);
          delay(BEEP_DURATION);
          toneAC(0);
#endif //MAKE_BEEP
        }
      }
      
      /* next */
      startMeasure();
    }
  }
}
