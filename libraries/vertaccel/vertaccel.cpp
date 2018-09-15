#include "vertaccel.h"
#include <Arduino.h>
#include <LightInvensense.h>

#ifdef VERTACCEL_ENABLE_EEPROM
#include <EEPROM.h>
#endif


/*-------------*/
/* calibration */
/*-------------*/

void Vertaccel::readCurrentDMPGyroCalibration(uint8_t* gyroCal) {

  fastMPUReadGyroBias(gyroCal);
}


#ifdef VERTACCEL_ENABLE_EEPROM
/******************/
/* EEPROM methods */
/******************/

void readEEPROMValues(int address, uint16_t eepromTag, int length, uint8_t* data) {

  /* read tag */
  uint16_t tag = EEPROM.read(address);
  address++;
  tag = (tag<<8) + EEPROM.read(address);
  address++;

  /* read values */
  for( uint8_t i = 0; i<length; i++) {
    if( tag == eepromTag ) {
      *data = EEPROM.read(address);
    } else {
      *data = 0;
    }
    data++;
    address++;
  }
}


void writeEEPROMValues(int address, uint16_t eepromTag, int length, const uint8_t* data) {

  /* write tag */
  EEPROM.write(address, (eepromTag>>8) & 0xff);
  EEPROM.write(address + 0x01, eepromTag & 0xff);
  address += 2;
  
  /* write values */
  for( uint8_t i = 0; i<length; i++) {
    EEPROM.write(address, *data);
    data++;
    address++;
  }
}


VertaccelSettings Vertaccel::readEEPROMSettings(void) {

  VertaccelSettings settings;
  readGyroCalibration(settings.gyroCal);
  readAccelCalibration(settings.accelCal);
#ifdef AK89xx_SECONDARY 
  readMagCalibration(settings.magCal);
#endif //AK89xx_SECONDARY

  return settings;
}


void Vertaccel::saveGyroCalibration(const uint8_t* gyroCal) {

  writeEEPROMValues(VERTACCEL_GYRO_CAL_EEPROM_ADDR, VERTACCEL_GYRO_CAL_EEPROM_TAG, 12, gyroCal);
}

void Vertaccel::readGyroCalibration(uint8_t* gyroCal) {

  readEEPROMValues(VERTACCEL_GYRO_CAL_EEPROM_ADDR, VERTACCEL_GYRO_CAL_EEPROM_TAG, 12, gyroCal);
}

void Vertaccel::saveAccelCalibration(const VertaccelCalibration& accelCal) {

  writeEEPROMValues(VERTACCEL_ACCEL_CAL_EEPROM_ADDR, VERTACCEL_ACCEL_CAL_EEPROM_TAG, sizeof(VertaccelCalibration), (uint8_t*)(&accelCal));
}

void Vertaccel::readAccelCalibration(VertaccelCalibration& accelCal) {

  readEEPROMValues(VERTACCEL_ACCEL_CAL_EEPROM_ADDR, VERTACCEL_ACCEL_CAL_EEPROM_TAG, sizeof(VertaccelCalibration), (uint8_t*)(&accelCal));
}

#ifdef AK89xx_SECONDARY
void Vertaccel::saveMagCalibration(const VertaccelCalibration& magCal) {

  writeEEPROMValues(VERTACCEL_MAG_CAL_EEPROM_ADDR, VERTACCEL_MAG_CAL_EEPROM_TAG, sizeof(VertaccelCalibration), (uint8_t*)(&magCal));
}

void Vertaccel::readMagCalibration(VertaccelCalibration& magCal) {

  readEEPROMValues(VERTACCEL_MAG_CAL_EEPROM_ADDR, VERTACCEL_MAG_CAL_EEPROM_TAG, sizeof(VertaccelCalibration), (uint8_t*)(&magCal));
}
#endif //AK89xx_SECONDARY
#endif //VERTACCEL_ENABLE_EEPROM
  


/***************/
/* init device */
/***************/
void Vertaccel::init(void) {

  /* init MPU */
  fastMPUInit(false);

  /* set gyro calibration in the DMP */
  fastMPUSetGyroBias(settings.gyroCal);

  /* set accel calibration in the DMP */
  int32_t accelBias[3];
  for(int i = 0; i<3; i++) 
    accelBias[i] = (int32_t)settings.accelCal.bias[i] << (15 - VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER);
  fastMPUSetAccelBiasQ15(accelBias);
  
  /* start DMP */
  fastMPUStart();
}


/* !!! WARNING : run as often as possible to check the FIFO stack !!! */
uint8_t Vertaccel::dataReady(void) {

  uint8_t newData = 0;
  int16_t iaccel[3];
  int32_t iquat[4];
   
  /* check if we have new data from imu */
  while( fastMPUReadFIFO(NULL, iaccel, iquat) >= 0 ) {
    newData = VERTACCEL_HAVE_ACCEL;
  }

  /* if new data compute vertical acceleration */
  if( newData ) {

    /*---------------------------------------*/
    /*   vertical acceleration computation   */
    /*---------------------------------------*/

    /***************************/
    /* normalize and calibrate */
    /***************************/
    double accel[3]; 
    double quat[4]; 

    for(int i = 0; i<3; i++) {
      int64_t calibratedAccel = (int64_t)iaccel[i] << VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER;
      calibratedAccel -= (int64_t)settings.accelCal.bias[i];
      calibratedAccel *= ((int64_t)settings.accelCal.scale + ((int64_t)1 << VERTACCEL_CAL_SCALE_MULTIPLIER));
      accel[i] = ((double)calibratedAccel)/((double)((int64_t)1 << (VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER + VERTACCEL_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT)));
    }

    
    for(int i = 0; i<4; i++)
      quat[i] = ((double)iquat[i])/LIGHT_INVENSENSE_QUAT_SCALE;
    
 
    /******************************/
    /* real and vert acceleration */
    /******************************/
  
    /* compute vertical direction from quaternions */
    double v[3];
    v[0] = 2*(quat[1]*quat[3]-quat[0]*quat[2]);
    v[1] = 2*(quat[2]*quat[3]+quat[0]*quat[1]);
    v[2] = 2*(quat[0]*quat[0]+quat[3]*quat[3])-1;
        
    /* compute real acceleration (without gravity) */
    double ra[3];
    for(int i = 0; i<3; i++) 
      ra[i] = accel[i] - v[i];
           
    /* compute vertical acceleration */
    vertAccel = (v[0]*ra[0] + v[1]*ra[1] + v[2]*ra[2]);

#ifdef AK89xx_SECONDARY
    /*-------------------------------*/
    /*   north vector computation    */
    /*-------------------------------*/
    if( fastMPUMagReady() ) {
      newData = VERTACCEL_HAVE_ACCEL | VERTACCEL_HAVE_MAG;

      /* get mag and calibrate */
      int16_t iNorth[3];
#ifdef VERTACCEL_USE_MAG_SENS_ADJ
      fastMPUReadMag(iNorth);
#else
      fastMPUReadRawMag(iNorth);
#endif //VERTACCEL_USE_MAG_SENS_ADJ

      double n[3];
      for(int i = 0; i<3; i++) {
	int64_t calibratedMag = ((int64_t)iNorth[i]) << VERTACCEL_MAG_CAL_BIAS_MULTIPLIER;
	calibratedMag -= (int64_t)settings.magCal.bias[i];
	calibratedMag *= ((int64_t)settings.magCal.scale + ((int64_t)1 << VERTACCEL_CAL_SCALE_MULTIPLIER));
	n[i] = ((double)calibratedMag)/((double)((int64_t)1 << (VERTACCEL_MAG_CAL_BIAS_MULTIPLIER + VERTACCEL_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT)));
      }
      
      /* compute north vector by applying rotation from v to z to vertor n */
      v[2] = -1.0 - v[2];
      northVector[0] = (1+v[0]*v[0]/v[2])*n[0] + (v[0]*v[1]/v[2])*n[1] - v[0]*n[2];
      northVector[1] = (v[0]*v[1]/v[2])*n[0] + (1+v[1]*v[1]/v[2])*n[1] - v[1]*n[2];
    }
#endif
  }
  
  return newData;
}


uint8_t Vertaccel::dataFullReady(double* accel, double* quat, double* v, double* n) {

  uint8_t newData = 0;
  int16_t iaccel[3];
  int32_t iquat[4];
   
  /* check if we have new data from imu */
  while( fastMPUReadFIFO(NULL, iaccel, iquat) >= 0 ) {
    newData = VERTACCEL_HAVE_ACCEL;
  }

  /* if new data compute vertical acceleration */
  if( newData ) {

    /*---------------------------------------*/
    /*   vertical acceleration computation   */
    /*---------------------------------------*/

    /***************************/
    /* normalize and calibrate */
    /***************************/
    for(int i = 0; i<3; i++) {
      int64_t calibratedAccel = (int64_t)iaccel[i] << VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER;
      calibratedAccel -= (int64_t)settings.accelCal.bias[i];
      calibratedAccel *= ((int64_t)settings.accelCal.scale + ((int64_t)1 << VERTACCEL_CAL_SCALE_MULTIPLIER));
      accel[i] = ((double)calibratedAccel)/((double)((int64_t)1 << (VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER + VERTACCEL_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT)));
    }

    
    for(int i = 0; i<4; i++)
      quat[i] = ((double)iquat[i])/LIGHT_INVENSENSE_QUAT_SCALE;
    
 
    /******************************/
    /* real and vert acceleration */
    /******************************/
  
    /* compute vertical direction from quaternions */
    v[0] = 2*(quat[1]*quat[3]-quat[0]*quat[2]);
    v[1] = 2*(quat[2]*quat[3]+quat[0]*quat[1]);
    v[2] = 2*(quat[0]*quat[0]+quat[3]*quat[3])-1;
        
    /* compute real acceleration (without gravity) */
    double ra[3];
    for(int i = 0; i<3; i++) 
      ra[i] = accel[i] - v[i];
           
    /* compute vertical acceleration */
    vertAccel = (v[0]*ra[0] + v[1]*ra[1] + v[2]*ra[2]);

#ifdef AK89xx_SECONDARY
    /*-------------------------------*/
    /*   north vector computation    */
    /*-------------------------------*/
    if( fastMPUMagReady() ) {
      newData = VERTACCEL_HAVE_ACCEL | VERTACCEL_HAVE_MAG;

      /* get mag and calibrate */
      int16_t iNorth[3];
#ifdef VERTACCEL_USE_MAG_SENS_ADJ
      fastMPUReadMag(iNorth);
#else
      fastMPUReadRawMag(iNorth);
#endif //VERTACCEL_USE_MAG_SENS_ADJ

      for(int i = 0; i<3; i++) {
	int64_t calibratedMag = ((int64_t)iNorth[i]) << VERTACCEL_MAG_CAL_BIAS_MULTIPLIER;
	calibratedMag -= (int64_t)settings.magCal.bias[i];
	calibratedMag *= ((int64_t)settings.magCal.scale + ((int64_t)1 << VERTACCEL_CAL_SCALE_MULTIPLIER));
	n[i] = ((double)calibratedMag)/((double)((int64_t)1 << (VERTACCEL_MAG_CAL_BIAS_MULTIPLIER + VERTACCEL_CAL_SCALE_MULTIPLIER + LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT)));
      }
      
      /* compute north vector by applying rotation from v to z to vertor n */
      double save = v[2];
      v[2] = -1.0 - v[2];
      northVector[0] = (1+v[0]*v[0]/v[2])*n[0] + (v[0]*v[1]/v[2])*n[1] - v[0]*n[2];
      northVector[1] = (v[0]*v[1]/v[2])*n[0] + (1+v[1]*v[1]/v[2])*n[1] - v[1]*n[2];
      v[2] = save;
    }
#endif
  }
  
  return newData;
}


/* when ready get data */
double Vertaccel::getValue(void) {

  return vertAccel * VERTACCEL_G_TO_MS;
}

#ifdef AK89xx_SECONDARY
double* Vertaccel::getNorthVector(void) {

  return northVector;
}
#endif //AK89xx_SECONDARY



/* direct access to sensors */
uint8_t Vertaccel::readRawAccel(int16_t* accel, int32_t* quat) {

  uint8_t haveValue = 0;

  while( fastMPUReadFIFO(NULL, accel, quat) >= 0 ) {
    haveValue = VERTACCEL_HAVE_ACCEL;
  }

  return haveValue;
}

  
#ifdef AK89xx_SECONDARY
uint8_t Vertaccel::readRawMag(int16_t* mag) {

  uint8_t haveValue = 0;

  if( fastMPUMagReady() ) {
#ifdef VERTACCEL_USE_MAG_SENS_ADJ
    fastMPUReadMag(mag);
#else
    fastMPUReadRawMag(mag);
#endif //VERTACCEL_USE_MAG_SENS_ADJ
    haveValue = VERTACCEL_HAVE_MAG;
  }

  return haveValue;
}  
#endif //AK89xx_SECONDARY

