#include <accelcalibrator.h>
#include <Arduino.h>
#include <vertaccel.h>

AccelCalibrator::AccelCalibrator() {

  /* init vars */
  for( int i=0; i<ACCEL_CALIBRATOR_ORIENTATION_COUNT; i++ ) {
    accelListDone[i] = false;
  }
  calibrated = false;
}

void AccelCalibrator::init(void) {
  
  /* init the accelerometer without auto calibration */
  vertaccel_init(false);

  /* get the values stored in EEPROM */
  double* cal = vertaccel_getCalibration();
  calibration[0] = cal[0];
  calibration[1] = cal[1];
  calibration[2] = cal[2];
}

void AccelCalibrator::reset(void) {
  
  for( int i=0; i<ACCEL_CALIBRATOR_ORIENTATION_COUNT; i++ ) {
    accelListDone[i] = false;
  }
  calibrated = false;
}
  
void AccelCalibrator::measure(void) {

  double accel[3];
  double upVector[3];
  double va;
  
  /* empty the FIFO and stabilize the accelerometer */
  unsigned long currentTime = millis();
  while( millis() - currentTime < ACCEL_CALIBRATOR_WAIT_DURATION ) {
    vertaccel_rawReady(accel, upVector, &va);
  }

  /* starting measures with mean filter */
  int count = 0;
  measuredAccel[0] = 0.0;
  measuredAccel[1] = 0.0;
  measuredAccel[2] = 0.0;
  double accelSquareMean[3] = {0.0, 0.0, 0.0}; //to compute standard deviation

  while( count < ACCEL_CALIBRATOR_FILTER_SIZE ) {
    if( vertaccel_rawReady(accel, upVector, &va) ) {
      measuredAccel[0] += accel[0];
      measuredAccel[1] += accel[1];
      measuredAccel[2] += accel[2];

      accelSquareMean[0] +=  accel[0]*accel[0];
      accelSquareMean[1] +=  accel[1]*accel[1];
      accelSquareMean[2] +=  accel[2]*accel[2];
    
      count++;
    }
  }

  /* compute mean accel */
  measuredAccel[0] /= (double)count;
  measuredAccel[1] /= (double)count;
  measuredAccel[2] /= (double)count;

  /* compute mean square */
  accelSquareMean[0] /= (double)count;
  accelSquareMean[1] /= (double)count;
  accelSquareMean[2] /= (double)count;

  /* compute standard deviation */
  measuredAccelSD = sqrt(accelSquareMean[0] - measuredAccel[0]*measuredAccel[0]);
  measuredAccelSD += sqrt(accelSquareMean[1] - measuredAccel[1]*measuredAccel[1]);
  measuredAccelSD += sqrt(accelSquareMean[2] - measuredAccel[2]*measuredAccel[2]);
}
    

int AccelCalibrator::getMeasureOrientation(void) {

  /* determine measure orientation */
  /* a>threshold -> 0                (a=1  correspond to 0) */
  /* -threshold < a < threshold -> 2 (a=0  correspond to 2) */
  /* a < -threshold -> 1             (a=-1 correspond to 1) */
  /* the choosen values is for computing the position in the list */
  int accelOrient[3];
  int zeroCount = 0;
  for( int i=0; i<3; i++ ) {
    if( measuredAccel[i] > ACCEL_CALIBRATOR_ORIENTATION_THRESHOLD ) {
      accelOrient[i] = 0;
    } else if( measuredAccel[i] > -ACCEL_CALIBRATOR_ORIENTATION_THRESHOLD ) {
      accelOrient[i] = 2;
      zeroCount++;
    } else {
      accelOrient[i] = 1;
    }
  }
  
  /* check if we have only one non zero accel (value 0 or 1) */
  if( zeroCount != 2 ) {
    return -1; //the orientation is ambiguous 
  }

  /* compute the position in the list */
  int orient = 0;
  int i = 0;
  while( accelOrient[i] == 2 )
    i++;

  while( i<3 ) {
    orient += accelOrient[i];
    i++;
  }

  return orient;
}


boolean AccelCalibrator::pushMeasure(void) {

  /* get orientation */
  int orientPos = this->getMeasureOrientation();
  if( orientPos < 0 ) 
    return false; //ambiguous orientation
 
  /* record the value */
  /* if a value exist, record only if better standard deviation */
  if( ! accelListDone[orientPos] || ( measuredAccelSD < accelListSD[orientPos] ) ) {
    accelList[orientPos * 3] = measuredAccel[0];
    accelList[orientPos * 3 + 1] = measuredAccel[1];
    accelList[orientPos * 3 + 2] = measuredAccel[2];
    accelListDone[orientPos] = true;
    accelListSD[orientPos] = measuredAccelSD;
    return true;
  }

  return false;
}

boolean AccelCalibrator::canCalibrate(void) {
  /* check if all the orientations except */
  /* the reversed position are done */
  for( int i=0; i<ACCEL_CALIBRATOR_ORIENTATION_COUNT; i++) {
    if( i != ACCEL_CALIBRATOR_ORIENTATION_EXCEPTION && !accelListDone[i] ) {
      return false;
    }
  }

  return true;  
}


void AccelCalibrator::calibrate(void) {

  /* check if we have enough measures */
  if( ! this->canCalibrate() )
    return;

  /****************************/
  /* make radius optimization */
  /****************************/
  double calibrationCenter[3];
  double baseRadius = ACCEL_CALIBRATOR_BASE_RADIUS;
  double baseRadiusDrift = ACCEL_CALIBRATOR_BASE_RADIUS_DRIFT;
  double baseStep = ACCEL_CALIBRATOR_BASE_RADIUS_STEP;
  double bestDistance = 100000.0;
  double bestRadius;
                        
  while( baseStep > ACCEL_CALIBRATOR_OPTIMIZATION_PRECISION ) {
              
    double currentRadius = baseRadius - baseRadiusDrift;
    double currentDistance;
              
    while( currentRadius <  baseRadius + baseRadiusDrift ) {
      computeCenter(&accelList[ACCEL_CALIBRATOR_ORIENTATION_P1*3],
		    &accelList[ACCEL_CALIBRATOR_ORIENTATION_P2*3],
		    &accelList[ACCEL_CALIBRATOR_ORIENTATION_P3*3],
		    currentRadius, calibrationCenter); 
      currentDistance = computeDistanceVariance(accelList, calibrationCenter);
      if( currentDistance <  bestDistance ) {
	bestDistance = currentDistance;
	bestRadius = currentRadius;
      }
      currentRadius += baseStep;
    }
    
    baseRadius = bestRadius;
    baseStep /= 10.0;
    baseRadiusDrift /= 10.0;
  }
            
  /* compute center with best radius */
  computeCenter(&accelList[ACCEL_CALIBRATOR_ORIENTATION_P1*3],
		&accelList[ACCEL_CALIBRATOR_ORIENTATION_P2*3],
		&accelList[ACCEL_CALIBRATOR_ORIENTATION_P3*3],
		bestRadius, calibrationCenter);

  /* save calibration */
  calibration[0] = -calibrationCenter[0];
  calibration[1] = -calibrationCenter[1];
  calibration[2] = -calibrationCenter[2];

  vertaccel_saveCalibration(calibration);
  calibrated = true;
}


void AccelCalibrator::calibratedMeasure(void) {

  this->measure();
  measuredAccel[0] += calibration[0];
  measuredAccel[1] += calibration[1];
  measuredAccel[2] += calibration[2];

}


/* given three vector and a radius, find the sphere's center */
void AccelCalibrator::computeCenter(double* v1, double* v2, double* v3, double radius, double* center) {
            
  /* compute midppoints */
  double m1[3];
  double m2[3];
      
  m1[0] = (v1[0]+v2[0])/2.0;
  m1[1] = (v1[1]+v2[1])/2.0;
  m1[2] = (v1[2]+v2[2])/2.0;
  m2[0] = (v1[0]+v3[0])/2.0;
  m2[1] = (v1[1]+v3[1])/2.0;
  m2[2] = (v1[2]+v3[2])/2.0;
      
  /* compute plane equations */
  double eq1[4];
  double eq2[4];
      
  eq1[0] = v2[0] - v1[0];
  eq1[1] = v2[1] - v1[1];
  eq1[2] = v2[2] - v1[2];
  eq1[3] = eq1[0]*m1[0] + eq1[1]*m1[1] + eq1[2]*m1[2];
      
  eq2[0] = v3[0] - v1[0];
  eq2[1] = v3[1] - v1[1];
  eq2[2] = v3[2] - v1[2];
  eq2[3] = eq2[0]*m2[0] + eq2[1]*m2[1] + eq2[2]*m2[2];
 
  /* simplify equ2 */
  double fac = eq2[0]/eq1[0];
  eq2[0] -= eq1[0]*fac;
  eq2[1] -= eq1[1]*fac;
  eq2[2] -= eq1[2]*fac;
  eq2[3] -= eq1[3]*fac;
  eq2[0] /= eq2[2]; 
  eq2[1] /= eq2[2];
  eq2[3] /= eq2[2];
  eq2[2] /= eq2[2];
      
  /* simplify equ1 */
  fac = eq1[2];
  eq1[1] -= eq2[1]*fac;
  eq1[2] -= eq2[2]*fac;
  eq1[3] -= eq2[3]*fac;
  eq1[1] /= eq1[0]; 
  eq1[2] /= eq1[0];
  eq1[3] /= eq1[0];
  eq1[0] /= eq1[0];
 
  /* get quadratic equation */
  double q[3];
  q[0]= eq1[1]*eq1[1] + 1 + eq2[1]*eq2[1];
  q[1]= 2*eq1[1]*(v1[0]-eq1[3])-2*v1[1]+2*eq2[1]*(v1[2]-eq2[3]);
  q[2]= (v1[0]-eq1[3])*(v1[0]-eq1[3]) + v1[1]*v1[1] + (v1[2]-eq2[3])*(v1[2]-eq2[3])-radius;
      
  /* solve quadratic */
  double d = q[1]*q[1] - 4*q[0]*q[2];
  double y1 = (-q[1]-sqrt(d))/(2*q[0]);
  double y2 = (-q[1]+sqrt(d))/(2*q[0]);
      
  /* compute points */
  if( -0.1 < y1 && y1 < 0.1 ) {      
    center[1] = y1;
    center[0] = eq1[3]-eq1[1]*y1;
    center[2] = eq2[3]-eq2[1]*y1;
  } else {
    center[1] = y2;
    center[0] = eq1[3]-eq1[1]*y2;
    center[2] = eq2[3]-eq2[1]*y2;
  }
}


/* given 6 vectors and a sphere center, compute distance from center variance */
double AccelCalibrator::computeDistanceVariance(double *v, double* center) {

  /* compute distances */
  double pointDistance[ACCEL_CALIBRATOR_ORIENTATION_COUNT];

  for( int i = 0; i<ACCEL_CALIBRATOR_ORIENTATION_COUNT; i++ ) {
    if( i != ACCEL_CALIBRATOR_ORIENTATION_EXCEPTION ) {
      double* val = &v[i*3];
      pointDistance[i] = sqrt( (val[0]-center[0])*(val[0]-center[0]) + (val[1]-center[1])*(val[1]-center[1]) + (val[2]-center[2])*(val[2]-center[2]) );
    }
  }
  
  /* compute mean */
  double mean = 0.0;
  
  for( int i = 0; i<ACCEL_CALIBRATOR_ORIENTATION_COUNT; i++ ) {
    if( i != ACCEL_CALIBRATOR_ORIENTATION_EXCEPTION ) {
      mean += pointDistance[i];
    }
  }
  mean /= ACCEL_CALIBRATOR_ORIENTATION_COUNT - 1;
  
  /* compute var */
  double var = 0.0;
  
  for( int i = 0; i<ACCEL_CALIBRATOR_ORIENTATION_COUNT; i++ ) {
    if( i != ACCEL_CALIBRATOR_ORIENTATION_EXCEPTION ) {
      var += (pointDistance[i] - mean)*(pointDistance[i] - mean);
    }
  }
  var /= ACCEL_CALIBRATOR_ORIENTATION_COUNT - 1;
  
  return var;
}



