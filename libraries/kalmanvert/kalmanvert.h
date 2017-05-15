#ifndef KALMANVERT_H
#define KALMANVERT_H

#include <Arduino.h>

/*********************************************************/
/* compute velocity from known position and acceleration */
/* p = position, v = velocity, a = acceleration          */
/*********************************************************/

class kalmanvert {

 public:
  /**********************************************************/
  /*           init with your first measured values         */
  /*  !!! sigmap and sigmaa are very important values !!!   */
  /* make experimentations : ex sigmap = 0.1 , sigmaa = 0.3 */
  /**********************************************************/
  void init(double startp, double starta, double sigmap, double sigmaa, unsigned long timestamp);

  /* run each time you get new values */
  void update(double mp, double ma, unsigned long timestamp);

  /* at any time get result */
  double getPosition();
  double getCalibratedPosition();
  double getVelocity();
  double getAcceleration();
  unsigned long getTimestamp();

  /* reset the current position without changing velocity and acceleration */
  void calibratePosition(double newPosition);

 private:
  /* position variance, acceleration variance */
  double varp, vara;
  
  /* position, velocity, acceleration, timestamp */
  double p, v, a;
  unsigned long t;

  /* calibration */
  double calibrationDrift;

  /* covariance matrix */
  double p11, p21, p12, p22;
  
};

#endif
