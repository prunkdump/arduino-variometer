#include "vertaccel.h"

#include <Arduino.h>

#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>


/******************/
/* data variables */
/******************/

static boolean newData;

/* vertical acceleration */
static double va;

/********************/
/* public functions */
/********************/

/* init vertaccel */
int vertaccel_init() {

  /* setting imu */
  mpu_select_device(0);
  mpu_init_structures();
  mpu_init(NULL);
  mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL); 
  mpu_set_gyro_fsr(VERTACCEL_GIRO_FSR);
  mpu_set_accel_fsr(VERTACCEL_ACCEL_FSR);
  mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL);

  /* setting dmp */
  dmp_select_device(0);
  dmp_init_structures();
  dmp_load_motion_driver_firmware();
  dmp_set_fifo_rate(VERTACCEL_FIFO_RATE);
  mpu_set_dmp_state(1);
  dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_RAW_ACCEL); 

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
  unsigned long timestamp;
  short sensors;
  unsigned char fifoCount;
  
  /* check if we have new data from imu */
  while( dmp_read_fifo(NULL,iaccel,iquat,&timestamp,&sensors,&fifoCount) == 0 ) {
    newData = true;
  }

  /* if new data compute vertical acceleration */
  if( newData ) {

    /***************************/
    /* normalize and calibrate */
    /***************************/
    double accel[3]; 
    double quat[4]; 
    
    accel[0] = ((double)iaccel[0])/VERTACCEL_ACCEL_SCALE + VERTACCEL_ACCEL_CORR_X;
    accel[1] = ((double)iaccel[1])/VERTACCEL_ACCEL_SCALE + VERTACCEL_ACCEL_CORR_Y;
    accel[2] = ((double)iaccel[2])/VERTACCEL_ACCEL_SCALE + VERTACCEL_ACCEL_CORR_Z;
        
    quat[0] = ((double)iquat[0])/VERTACCEL_QUAT_SCALE;
    quat[1] = ((double)iquat[1])/VERTACCEL_QUAT_SCALE;
    quat[2] = ((double)iquat[2])/VERTACCEL_QUAT_SCALE;
    quat[3] = ((double)iquat[3])/VERTACCEL_QUAT_SCALE;
    

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


/* must be run before reading values */
void vertaccel_updateData() {

  /* start a new acceleration computation */
  newData = false;
}


/* get vertical acceleration */
double vertaccel_getValue() {
  return va * VERTACCEL_G_TO_MS;
}






  
