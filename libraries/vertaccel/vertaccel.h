#ifndef VERTACCEL_H
#define VERTACCEL_H

#include <Arduino.h>

#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>

#define VERTACCEL_G_TO_MS 9.80665

#define VERTACCEL_GIRO_FSR 2000
#define VERTACCEL_ACCEL_FSR 4
#define VERTACCEL_FIFO_RATE 100

/* 4G ~= 2^15 */
#define VERTACCEL_ACCEL_SCALE 8192.0
#define VERTACCEL_ACCEL_CORR_X (+0.00415)
#define VERTACCEL_ACCEL_CORR_Y (-0.0137)
#define VERTACCEL_ACCEL_CORR_Z (+0.0141)

/* 2^30 */
#define VERTACCEL_QUAT_SCALE 1073741824.0


/*******************/
/* init the device */
/*******************/
int vertaccel_init();

/***********/
/* reading */
/***********/

/* !!! WARNING : run as often as possible to clear the FIFO stack !!! */
boolean vertaccel_dataReady();

/* when ready update and get data */
void vertaccel_updateData();
double vertaccel_getValue();

#endif
