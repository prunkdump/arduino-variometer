#ifndef BEEPER_H
#define BEEPER_H

#define BEEP_VELOCITY_LOWER_THRESHOLD (-2.0)
#define BEEP_VELOCITY_UPPER_THRESHOLD 0.20

/* avoid changing beep freq too often */
#define BEEP_VELOCITY_SENSITIVITY 0.1

/* length of beep in vertical meters */ 
#define BEEP_HIGH_LENGTH 0.16
#define BEEP_LOW_LENGTH 0.04

/* beep sound freq computation : BEEP_FREQ_COEFF * velocity + BEEP_BASE_FREQ */
#define BEEP_BASE_FREQ 1000.0
#define BEEP_FREQ_COEFF 150.0
#define BEEP_LOW_FREQ 100.0

/* beep velocity filter */
/* filteredVelocity = beepVelocity * BEEP_VELOCITY_FILTER_COEFF + BEEP_VELOCITY_FILTER_BASE */
#define BEEP_VELOCITY_FILTER_BASE 0.1
#define BEEP_VELOCITY_FILTER_COEFF 0.5


class beeper {

 public:
  beeper(double lowerThreshold = BEEP_VELOCITY_LOWER_THRESHOLD, double upperThreshold = BEEP_VELOCITY_UPPER_THRESHOLD);

  /* to stop beeper, set a very low and hight threshold. ex : -1000.0 and 1000.0 */
  void setThresholds(double lowerThreshold = BEEP_VELOCITY_LOWER_THRESHOLD, double upperThreshold = BEEP_VELOCITY_UPPER_THRESHOLD);

  /* run each time you get new velocity data */
  void setVelocity(double velocity);

  /* run as often as possible */
  void update();

 private:
  void setBeepParameters(double velocity);
  double beepLowerThreshold;
  double beepUpperThreshold;
  unsigned long beepStartTime;
  double beepVelocity;
  double beepFreq;
  boolean beepPaternEnabled;
  double beepPaternBasePosition;
  double beepPaternPosition;
  boolean beepStatus;
};


#endif
