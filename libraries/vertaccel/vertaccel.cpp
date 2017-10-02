#include "vertaccel.h"

#include <Arduino.h>

#include <SparkFunMPU9250-DMP.h>

#include <FlashAsEEPROM.h>

#include <VarioSettings.h>

MPU9250_DMP imu;

/******************/
/* data variables */
/******************/

/* calibration */
static double accelCal[3];

static boolean newData;

/* vertical acceleration */
static double va;


/*************************/
/* calibration functions */
/*************************/

/* read calibration available from EEPROM */
boolean vertaccel_readAvailableCalibration(void) {

  /* check tag */
  uint16_t eepromTag;
  
  if (!EEPROM.isValid()) {
     Serial.println("EEPROM is empty, writing some example data:");
 	return false;
  }
  else { 
    eepromTag = EEPROM.read(VERTACCEL_EPROM_ADDR);
    eepromTag <<= 8;
    eepromTag += EEPROM.read(VERTACCEL_EPROM_ADDR + 0x01);
  
    if( eepromTag != VERTACCEL_EPROM_TAG ) { return false;  } 
    else { return true;  }
  }
}

/* read calibration from EEPROM */
void vertaccel_readCalibration(void) {

  /* check tag */
  uint16_t eepromTag;
  
 if (!EEPROM.isValid()) {
    Serial.println("EEPROM is empty, writing some example data:");
	if (GnuSettings.readFlashSDSettings() == true) {
	  if ((GnuSettings.ACCELCALX ==0) && (GnuSettings.ACCELCALY ==0) &&	(GnuSettings.ACCELCALZ ==0)) {
	    accelCal[0] = 0.0;
        accelCal[1] = 0.0;
        accelCal[2] = 0.0; 
 
	  }
	  else {
		accelCal[0] = GnuSettings.ACCELCALX;
        accelCal[1] = GnuSettings.ACCELCALY;
        accelCal[2] = GnuSettings.ACCELCALZ; 
	  }
	}
	else {
      accelCal[0] = 0.0;
      accelCal[1] = 0.0;
      accelCal[2] = 0.0; 
	}
 }
 else { 
   eepromTag = EEPROM.read(VERTACCEL_EPROM_ADDR);
   eepromTag <<= 8;
   eepromTag += EEPROM.read(VERTACCEL_EPROM_ADDR + 0x01);
  
  if( eepromTag != VERTACCEL_EPROM_TAG ) {
     accelCal[0] = 0.0;
     accelCal[1] = 0.0;
     accelCal[2] = 0.0;
   } else {
     /* read calibration settings */
     uint8_t* datap = (uint8_t*)accelCal;
     for( unsigned i = 0; i<sizeof(accelCal); i++ ) {
       datap[i] =  EEPROM.read(VERTACCEL_EPROM_ADDR + 0x02 + i);
     }
   }
 }
}

/* save calibration to EEPROM */
void vertaccel_saveCalibration(double* cal) {

  /* write tag */
  uint16_t eepromTag = VERTACCEL_EPROM_TAG;
  EEPROM.write(VERTACCEL_EPROM_ADDR, (eepromTag>>8) & 0xff);
  EEPROM.write(VERTACCEL_EPROM_ADDR + 0x01, eepromTag & 0xff);
  EEPROM.commit();

  
  /* save calibration settings */
  uint8_t* datap = (uint8_t*)cal;
  for( unsigned i = 0; i<3*sizeof(double); i++ ) {
    EEPROM.write(VERTACCEL_EPROM_ADDR + 0x02 + i, datap[i]);
  }

  EEPROM.commit();
  
  /* save in global var */
  accelCal[0] = cal[0];
  accelCal[1] = cal[1];
  accelCal[2] = cal[2];
  
  GnuSettings.ACCELCALX = cal[0];
  GnuSettings.ACCELCALY = cal[1];
  GnuSettings.ACCELCALZ = cal[2];
  GnuSettings.writeFlashSDSettings();
} 

/* give calibration coefficients */
double* vertaccel_getCalibration(void) {

  return accelCal;
}  
 

/********************/
/* public functions */
/********************/

/* init vertaccel */
void vertaccel_init(boolean giroCalibration) {

 #ifdef IMU_DEBUG
   SerialPort.println("vertaccel begin");
 #endif //IMU_BEBUG 

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
  
 #ifdef IMU_DEBUG
   SerialPort.println("imu Begin");
 #endif //IMU_BEBUG 
    delay(8000);
 
  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              100); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
  /* init calibration settings */
 
 #ifdef IMU_DEBUG
   SerialPort.println("dmp Begin");
 #endif //IMU_BEBUG 
 
 vertaccel_readCalibration();
  
  /* init data variables */
  newData = false;
}

void vertaccel_initimu(void) {

 #ifdef IMU_DEBUG
   SerialPort.println("vertaccel begin imu");
 #endif //IMU_BEBUG 

  // Call imu.begin() to verify communication and initialize
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }
 
#ifdef IMU_DEBUG
   SerialPort.println("dmp Begin");
#endif //IMU_BEBUG 

}

void vertaccel_initdmp(boolean giroCalibration) {

 #ifdef IMU_DEBUG
   SerialPort.println("vertaccel begin dmp");
 #endif //IMU_BEBUG 

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
              100); // Set DMP FIFO rate to 10 Hz
  // DMP_FEATURE_LP_QUAT can also be used. It uses the 
  // accelerometer in low-power mode to estimate quat's.
  // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
  /* init calibration settings */
 
 #ifdef IMU_DEBUG
   SerialPort.println("dmp Begin");
 #endif //IMU_BEBUG 
 
 vertaccel_readCalibration();
  
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
/*  unsigned long timestamp;
  short sensors;
  unsigned char fifoCount;
  
 //  Serial.println("dataready");

  /* check if we have new data from imu *
  while( dmp_read_fifo(NULL,iaccel,iquat,&timestamp,&sensors,&fifoCount) == 0 ) {
    newData = true;
  }*/
  
   // Then read while there is data in the FIFO
   
  if ( imu.fifoAvailable() )
    {
    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
      if ( imu.dmpUpdateFifo() == INV_SUCCESS)
        {
		  newData = true;
        }
    }

 /*  while ( imu.fifoAvailable() == 0)
        {
          // Call updateFifo to update ax, ay, az, gx, gy, and/or gz
          if ( imu.dmpUpdateFifo() == INV_SUCCESS)
            {
			  newData = true;
            }
        }*/
    


  /* if new data compute vertical acceleration */
  if( newData ) {

 //  Serial.println("new data");

    /***************************/
    /* normalize and calibrate */
    /***************************/
    double accel[3]; 
    double quat[4]; 
    
/*    accel[0] = ((double)iaccel[0])/VERTACCEL_ACCEL_SCALE + accelCal[0];
    accel[1] = ((double)iaccel[1])/VERTACCEL_ACCEL_SCALE + accelCal[1];
    accel[2] = ((double)iaccel[2])/VERTACCEL_ACCEL_SCALE + accelCal[2];*/

    accel[0] = ((double)imu.ax)/VERTACCEL_ACCEL_SCALE + accelCal[0];
    accel[1] = ((double)imu.ay)/VERTACCEL_ACCEL_SCALE + accelCal[1];
    accel[2] = ((double)imu.az)/VERTACCEL_ACCEL_SCALE + accelCal[2];
	
 /*   quat[0] = imu.calcQuat(imu.qw);     //((double)iquat[0])/VERTACCEL_QUAT_SCALE;
    quat[1] = imu.calcQuat(imu.qx);     //((double)iquat[1])/VERTACCEL_QUAT_SCALE;
    quat[2] = imu.calcQuat(imu.qy);     //((double)iquat[2])/VERTACCEL_QUAT_SCALE;
    quat[3] = imu.calcQuat(imu.qz);     //((double)iquat[3])/VERTACCEL_QUAT_SCALE;*/
 
    quat[0] = ((double)imu.qw)/VERTACCEL_QUAT_SCALE;
    quat[1] = ((double)imu.qx)/VERTACCEL_QUAT_SCALE;
    quat[2] = ((double)imu.qy)/VERTACCEL_QUAT_SCALE;
    quat[3] = ((double)imu.qz)/VERTACCEL_QUAT_SCALE;

 
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
/*  unsigned long timestamp;
  short sensors;
  unsigned char fifoCount;
  
  /* check if we have new data from imu *
  while( dmp_read_fifo(NULL,iaccel,iquat,&timestamp,&sensors,&fifoCount) == 0 ) {
    newData = true;
  }*/
  
    if ( imu.fifoAvailable() >= 256)
    {
      // Then read while there is data in the FIFO
      while ( imu.fifoAvailable() > 0)
        {
          // Call updateFifo to update ax, ay, az, gx, gy, and/or gz
          if ( imu.dmpUpdateFifo() == INV_SUCCESS)
            {
			  newData = true;
            }
        }
    }



  /* if new data compute vertical acceleration */
  if( newData ) {

    /*************/
    /* normalize */
    /*************/ 
    double quat[4]; 
    
 /*   accel[0] = ((double)iaccel[0])/VERTACCEL_ACCEL_SCALE;
    accel[1] = ((double)iaccel[1])/VERTACCEL_ACCEL_SCALE;
    accel[2] = ((double)iaccel[2])/VERTACCEL_ACCEL_SCALE;
        
    quat[0] = ((double)iquat[0])/VERTACCEL_QUAT_SCALE;
    quat[1] = ((double)iquat[1])/VERTACCEL_QUAT_SCALE;
    quat[2] = ((double)iquat[2])/VERTACCEL_QUAT_SCALE;
    quat[3] = ((double)iquat[3])/VERTACCEL_QUAT_SCALE;*/
    
    accel[0] = ((double)imu.ax)/VERTACCEL_ACCEL_SCALE;
    accel[1] = ((double)imu.ay)/VERTACCEL_ACCEL_SCALE;
    accel[2] = ((double)imu.az)/VERTACCEL_ACCEL_SCALE;
	
    quat[0] = ((double)imu.qw)/VERTACCEL_QUAT_SCALE;
    quat[1] = ((double)imu.qx)/VERTACCEL_QUAT_SCALE;
    quat[2] = ((double)imu.qy)/VERTACCEL_QUAT_SCALE;
    quat[3] = ((double)imu.qz)/VERTACCEL_QUAT_SCALE;

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
