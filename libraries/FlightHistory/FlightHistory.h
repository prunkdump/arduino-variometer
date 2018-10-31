#ifndef FLIGHT_HISTORY_H
#define FLIGHT_HISTORY_H


/*******************************/
/* base class for alti history */
/*******************************/
template<unsigned period, int8_t count>
class FlightHistory {

 public:
 FlightHistory(void) : altiValuesSize(count + 1), altiValuesPos(0), haveClimbRate(false) { }
    
  void init(double firstAlti, unsigned long timestamp);
  void setAlti(double alti, unsigned long timestamp);
  bool haveNewClimbRate(void);
  double getClimbRate(void);

 protected:
  void saveAlti(double alti);
  double computeClimbDelta(void);
  
  const int8_t altiValuesSize;
  int8_t altiValuesPos;
  double altiValues[count + 1];
  double lastAlti;
  unsigned long lastAltiTimestamp;
  unsigned long lastSaveTimestamp;
  bool haveClimbRate;

};

template<unsigned period, int8_t count>
  void FlightHistory<period, count>::init(double firstAlti, unsigned long timestamp) {
  for( int8_t i = 0; i<altiValuesSize; i++ ) {
    altiValues[i] = firstAlti;
  }

  lastAltiTimestamp = timestamp;
  lastSaveTimestamp = timestamp;
}


template<unsigned period, int8_t count>
void FlightHistory<period, count>::saveAlti(double alti) {

  altiValues[altiValuesPos] = alti;
  altiValuesPos = (altiValuesPos + 1) % altiValuesSize;

  /* we can compute a new climp rate */
  haveClimbRate = true;
}


template<unsigned period, int8_t count>
void FlightHistory<period, count>::setAlti(double alti, unsigned long timestamp) {

  /* check if time elasped */
  if( timestamp <= lastAltiTimestamp || timestamp <= lastSaveTimestamp )
    return;
  
  /* check if we need to save alti */
  if( timestamp - lastSaveTimestamp >= period ) {

    /* udpate timestamp */
    lastSaveTimestamp += period; //is now the save timestamp !!
    
    double newAlti;
    if( lastSaveTimestamp <= lastAltiTimestamp ) {
      newAlti = lastAlti;
    } else if( lastSaveTimestamp >= timestamp ) {
      newAlti = alti;
    } else {
      /*compute linear interpolated alti */
      unsigned long remainingTime = lastSaveTimestamp - lastAltiTimestamp;
      newAlti = lastAlti + ((alti - lastAlti)*(double)(remainingTime)/(double)(timestamp - lastAltiTimestamp));
    }
    
    /* save */
    saveAlti(newAlti);
  }

  lastAlti = alti;
  lastAltiTimestamp = timestamp;
}


template<unsigned period, int8_t count>
bool FlightHistory<period, count>::haveNewClimbRate(void) {

  return haveClimbRate;
}


template<unsigned period, int8_t count>
double FlightHistory<period, count>::computeClimbDelta(void) {

  /* get last alti position  */
  int8_t previousPos = altiValuesPos - 1;
  if( previousPos < 0 )
    previousPos = altiValuesSize - 1;

  /* compute climb delta */
  return altiValues[previousPos] - altiValues[altiValuesPos];
}

 
template<unsigned period, int8_t count>
double FlightHistory<period, count>::getClimbRate(void) {

  haveClimbRate = false;
  return computeClimbDelta()*1000.0/(double)(count * period);
}


/************************************/
/* derived class with speed history */
/************************************/
template<unsigned period, int8_t count, int8_t speedPeriodCount>
class SpeedFlightHistory : public FlightHistory<period, count> {

  using FlightHistory<period, count>::computeClimbDelta;
  using FlightHistory<period, count>::lastAlti;
  using FlightHistory<period, count>::lastSaveTimestamp;
  using FlightHistory<period, count>::saveAlti;

 public:
 SpeedFlightHistory(void) : speedValuesSize( (count + speedPeriodCount - 1)/speedPeriodCount ), speedValuesPos(0) { }
  double getGlideRatio(double speed, unsigned long timestamp);

 private:
  const int8_t speedValuesSize;
  int8_t speedValuesPos;
  double speedValues[ (count + speedPeriodCount - 1)/speedPeriodCount  ];
};


template<unsigned period, int8_t count, int8_t speedPeriodCount>
double SpeedFlightHistory<period, count, speedPeriodCount>::getGlideRatio(double speed, unsigned long timestamp) {

  /* save speed value */
  speedValues[speedValuesPos] = speed;
  speedValuesPos = (speedValuesPos + 1) % speedValuesSize;

  /*******************************/
  /* synchronise alti with speed */
  /*******************************/
  
  /* if we don't have the alti measure yet */
  /* take lastAlti                         */
  if( timestamp > lastSaveTimestamp && (timestamp - lastSaveTimestamp) > period/2 ) {

    /* add alti value */
    saveAlti(lastAlti);
  }

  /* sync */
  lastSaveTimestamp = timestamp;

  
  /***********************/
  /* compute glide ratio */
  /***********************/
  unsigned sumCount = 0;

  double distance = 0.0;
  int8_t speedPos = speedValuesPos;

  while( sumCount < count ) {

    /* check if we need to update speed pos */
    if( sumCount % speedPeriodCount == 0 ) {
      
      speedPos -= 1;
      if( speedPos < 0 ) { 
	speedPos = speedValuesSize - 1;
      }
    }

    /* add distance */
    distance += speedValues[speedPos]; 

    /* next period */
    sumCount++;
  }

  return -distance*((double)period/3600.0)/computeClimbDelta(); //speed is in km/h and period in ms, so we get m
 }

#endif
