/* accel_calibration_nointeractive -- Accelerometer calibration
 *                                    without serial communication
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
#include <IntTW.h>
#include <vertaccel.h>
#include <EEPROM.h>
#include <LightInvensense.h>
#include <avr/pgmspace.h>
#include <accelcalibrator.h>
#include <toneAC.h>
#include <FirmwareUpdater.h>

#define HIGH_BEEP_FREQ 1000.0
#define LOW_BEEP_FREQ 100.0
#define BASE_BEEP_DURATION 100.0

#define MEASURE_DELAY 4000

AccelCalibrator calibrator;

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


void setup() {
  
  /* init calibrator */
  delay(VARIOMETER_POWER_ON_DELAY);
  intTW.begin();
  calibrator.init();
  
  /* launch firmware update if needed */
  if( firmwareUpdateCond() ) {
   firmwareUpdate();
  }
    
  /* start beep */
  signalBeep(HIGH_BEEP_FREQ, BASE_BEEP_DURATION, 3);

  /* wait for gyro calibration */
  delay(10000);
  signalBeep(HIGH_BEEP_FREQ, BASE_BEEP_DURATION, 2);

}

void loop() {
  
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
}
