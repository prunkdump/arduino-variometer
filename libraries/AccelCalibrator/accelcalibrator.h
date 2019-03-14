/* accelcalibrator -- Calibrate accelerometer from real time measures
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

#ifndef ACCEL_CALIBRATOR_H
#define ACCEL_CALIBRATOR_H

#include <Arduino.h>
#include <vertaccel.h>

#define ACCEL_CALIBRATOR_ORIENTATION_COUNT 6
#define ACCEL_CALIBRATOR_ORIENTATION_THRESHOLD 0.7

/*********************/
/*    ORIENTATION    */
/*********************/

/* here the accel orientation code : */
/* (0, 0, 1)  ->  0                  */
/* (0, 0, -1) ->  1                  */
/* (0, 1, 0)  ->  2                  */
/* (0, -1, 0) ->  3                  */
/* (1, 0, 0)  ->  4                  */
/* (-1, 0, 0) ->  5                  */

/* !!! adjust the following settings to your setup !!! */ 

/* -> the exception  */
/* this orientation is not used in the calibration procedure */
/* because the accelerometer is almost never in this position */
/* usually pointing to the the ground */
#define ACCEL_CALIBRATOR_ORIENTATION_EXCEPTION 1

/* -> the three prority orientations */
/* the three orientations used for base calibration */
/* this make priority to them. You may use : */
/* toward sky, toward left or right, toward user */
#define ACCEL_CALIBRATOR_ORIENTATION_P1 0
#define ACCEL_CALIBRATOR_ORIENTATION_P2 3
#define ACCEL_CALIBRATOR_ORIENTATION_P3 4


/**************************/
/* CALIBRATION PARAMETERS */
/**************************/
#define ACCEL_CALIBRATOR_WAIT_DURATION 1000
#define ACCEL_CALIBRATOR_FILTER_SIZE 300
#define ACCEL_CALIBRATOR_BASE_RADIUS 1.0
#define ACCEL_CALIBRATOR_BASE_RADIUS_DRIFT 0.01
#define ACCEL_CALIBRATOR_BASE_RADIUS_STEP 0.001
#define ACCEL_CALIBRATOR_OPTIMIZATION_PRECISION 0.000001


class AccelCalibrator {

 public:
  /* for one measure */
  double measuredAccel[3];  //computed by measure()
  double measuredAccelSD;   //accel standard deviation

  /* store measure for calibration */
  double accelList[3*ACCEL_CALIBRATOR_ORIENTATION_COUNT]; //all possible orientations
  boolean accelListDone[ACCEL_CALIBRATOR_ORIENTATION_COUNT];
  double accelListSD[ACCEL_CALIBRATOR_ORIENTATION_COUNT]; //record strandard deviation
  
  /* final calibration data */
  double calibration[3]; 
  boolean calibrated;

    AccelCalibrator();

  void init(void);
    
  /* reset the calibration, you need to restart measures */
  void reset(void);

  /* launch accel measure and compute standard deviation */
  void measure(void);

  /* return measure orientation */
  int getMeasureOrientation(void);

  /* save the measured value */
  /* return if the measure can be used for calibration */
  /* possible problems : ambiguous orientation, bad standard deviation */
  boolean pushMeasure(void);

  /* once all measures are done, calibrate */
  boolean canCalibrate(void);
  void calibrate(void);

  /* you can get calibrated measures */
  void calibratedMeasure(void);
  

 private:
  Vertaccel vertaccel;
  void computeCenter(double* v1, double* v2, double* v3, double radius, double* center);
  double computeDistanceVariance(double *v, double* center);

};

#endif
