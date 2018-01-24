#include "vertaccel.h"

#include <Arduino.h>

#include <LightInvensense.h>
#include <EEPROM.h>

/******************/
/* data variables */
/******************/

/* calibration */
static short accelCal[3];
#ifdef AK89xx_SECONDARY
static short magCal[3];
#endif

static boolean newData;

/* vertical acceleration */
static double va;


/*************************/
/* calibration functions */
/*************************/


void readEEPROMValues(int address, uint16_t eepromTag, short* buff) {

  /* read tag */
  uint16_t tag = EEPROM.read(address);
  address++;
  tag = (tag<<8) + EEPROM.read(address);
  address++;

  /* read values */
  uint8_t* data = (uint8_t*)buff;
  for( uint8_t i = 0; i<3*sizeof(short); i++) {
    if( tag == eepromTag ) {
      *data = EEPROM.read(address);
    } else {
      *data = 0;
    }
    data++;
    address++;
  }
}

void writeEEPROMValues(int address, uint16_t eepromTag, short* buff) {

  /* write tag */
  EEPROM.write(address, (eepromTag>>8) & 0xff);
  EEPROM.write(address + 0x01, eepromTag & 0xff);
  address += 2;
  
  /* write values */
  uint8_t* data = (uint8_t*)buff;
  for( uint8_t i = 0; i<3*sizeof(short); i++) {
    EEPROM.write(address, *data);
    data++;
    address++;
  }
}
  

/* read accel calibration from EEPROM */
void vertaccel_readAccelCalibration(void) {

  readEEPROMValues(VERTACCEL_ACCEL_EEPROM_ADDR, VERTACCEL_ACCEL_EEPROM_TAG, accelCal);
}

/* save accel calibration to EEPROM */
void vertaccel_saveAccelCalibration(double* cal) {

  for( uint8_t i = 0; i<3; i++ ) { 
    accelCal[i] = (short)(cal[i] * LIGHT_INVENSENSE_ACCEL_SCALE);
  }

  writeEEPROMValues(VERTACCEL_ACCEL_EEPROM_ADDR, VERTACCEL_ACCEL_EEPROM_TAG, accelCal);
}

#ifdef AK89xx_SECONDARY
/* read mag calibration from EEPROM */
void vertaccel_readMagCalibration(void) {

  readEEPROMValues(VERTACCEL_MAG_EEPROM_ADDR, VERTACCEL_MAG_EEPROM_TAG, magCal);
}

/* save mag calibration to EEPROM */
void vertaccel_saveMagCalibration(double* cal) {

  
  for( uint8_t i = 0; i<3; i++ ) { 
    magCal[i] = (short)cal[i];
  }

  writeEEPROMValues(VERTACCEL_MAG_EEPROM_ADDR, VERTACCEL_MAG_EEPROM_TAG, magCal);
}
#endif

/* give accel calibration coefficients */
void vertaccel_getAccelCalibration(double* cal) {
  
  for( uint8_t i = 0; i<3; i++ ) { 
    cal[i] = (double)accelCal[i] / LIGHT_INVENSENSE_ACCEL_SCALE;
  }
}

#ifdef AK89xx_SECONDARY
/* give mag calibration coefficients */
void vertaccel_getMagCalibration(double* cal) {

  for( uint8_t i = 0; i<3; i++ ) { 
    cal[i] = (double)magCal[i];
  }
}
#endif




/********************/
/* public functions */
/********************/

/* init vertaccel */
void vertaccel_init(void) {

  /* init */
  fastMPUInit();

  /* init calibration settings */
  vertaccel_readAccelCalibration();
  vertaccel_readMagCalibration();

  /* init data variables */
  newData = false;
}

/* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
/*   must be run as often as possible   */
/* check if data ready and threat data  */
/* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
boolean vertaccel_dataReady() {
  
  short iaccel[3];
  long iquat[4];
   
  /* check if we have new data from imu */
  while( fastMPUReadFIFO(NULL, iaccel, iquat) >= 0 ) {
    newData = true;
  }

  /* if new data compute vertical acceleration */
  if( newData ) {

    /***************************/
    /* normalize and calibrate */
    /***************************/
    double accel[3]; 
    double quat[4]; 
    
    accel[0] = ((double)(iaccel[0]+ accelCal[0]))/LIGHT_INVENSENSE_ACCEL_SCALE;
    accel[1] = ((double)(iaccel[0]+ accelCal[1]))/LIGHT_INVENSENSE_ACCEL_SCALE;
    accel[2] = ((double)(iaccel[0]+ accelCal[2]))/LIGHT_INVENSENSE_ACCEL_SCALE;
        
    quat[0] = ((double)iquat[0])/LIGHT_INVENSENSE_QUAT_SCALE;
    quat[1] = ((double)iquat[1])/LIGHT_INVENSENSE_QUAT_SCALE;
    quat[2] = ((double)iquat[2])/LIGHT_INVENSENSE_QUAT_SCALE;
    quat[3] = ((double)iquat[3])/LIGHT_INVENSENSE_QUAT_SCALE;
    

    /******************************/
    /* real and vert acceleration */
    /******************************/
  
    /* compute upper direction from quaternions */
    double ux, uy, uz;
    ux = 2*(quat[1]*quat[3]-quat[0]*quat[2]);
    uy = 2*(quat[2]*quat[3]+quat[0]*quat[1]);
    uz = 2*(quat[0]*quat[0]+quat[3]*quat[3])-1;
        
    /* compute real acceleration (without gravity) */
    double rax, ray, raz;
    rax = accel[0] - ux;
    ray = accel[1] - uy;
    raz = accel[2] - uz;
    
        
    /* compute vertical acceleration */
    va = (ux*rax + uy*ray + uz*raz);
  }
 
  return newData;
}


/* send raw data for debugging or calibration */ 
boolean vertaccel_rawReady(double* accel, double* upVector, double* vertAccel) {
  
  short iaccel[3];
  long iquat[4];
   
  /* check if we have new data from imu */
  while( fastMPUReadFIFO(NULL, iaccel, iquat) >= 0 ) {
    newData = true;
  }

  /* if new data compute vertical acceleration */
  if( newData ) {
    
    /*************/
    /* normalize */
    /*************/ 
    double quat[4]; 
    
    accel[0] = ((double)iaccel[0])/LIGHT_INVENSENSE_ACCEL_SCALE;
    accel[1] = ((double)iaccel[1])/LIGHT_INVENSENSE_ACCEL_SCALE;
    accel[2] = ((double)iaccel[2])/LIGHT_INVENSENSE_ACCEL_SCALE;
        
    quat[0] = ((double)iquat[0])/LIGHT_INVENSENSE_QUAT_SCALE;
    quat[1] = ((double)iquat[1])/LIGHT_INVENSENSE_QUAT_SCALE;
    quat[2] = ((double)iquat[2])/LIGHT_INVENSENSE_QUAT_SCALE;
    quat[3] = ((double)iquat[3])/LIGHT_INVENSENSE_QUAT_SCALE;
    

    /******************************/
    /* real and vert acceleration */
    /******************************/
  
    /* compute upper direction from quaternions */
    double ux, uy, uz;
    ux = 2*(quat[1]*quat[3]-quat[0]*quat[2]);
    uy = 2*(quat[2]*quat[3]+quat[0]*quat[1]);
    uz = 2*(quat[0]*quat[0]+quat[3]*quat[3])-1;
        
    /* compute real acceleration (without gravity) */
    double rax, ray, raz;
    rax = accel[0] - ux;
    ray = accel[1] - uy;
    raz = accel[2] - uz;
    
        
    /* compute vertical acceleration */
    upVector[0] = ux;
    upVector[1] = uy;
    upVector[2] = uz;
    *vertAccel = (ux*rax + uy*ray + uz*raz);
  }
 
  return newData;
}


/* must be run before reading values */
void vertaccel_updateData() {

  /* start a new acceleration computation */
  newData = false;
}


/* get vertical acceleration */
double vertaccel_getValue() {
  return va * VERTACCEL_G_TO_MS;
}
