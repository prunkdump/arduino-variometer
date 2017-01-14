#ifndef VERTACCEL_H
#define VERTACCEL_H

#include <Arduino.h>

#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <EEPROM.h>

/* eeprom calibration adresses */
#define VERTACCEL_EPROM_TAG 9806
#define VERTACCEL_EPROM_ADDR 0x00

/* accelerometer parameters */
#define VERTACCEL_G_TO_MS 9.80665

#define VERTACCEL_GIRO_FSR 2000
#define VERTACCEL_ACCEL_FSR 4
#define VERTACCEL_FIFO_RATE 100

/* 4G ~= 2^15 */
#define VERTACCEL_ACCEL_SCALE 8192.0


/* 2^30 */
#define VERTACCEL_QUAT_SCALE 1073741824.0


/*******************/
/* init the device */
/*******************/
int vertaccel_init(boolean giroCalibration = true);

/***********/
/* reading */
/***********/

/* !!! WARNING : run as often as possible to clear the FIFO stack !!! */
boolean vertaccel_dataReady();
boolean vertaccel_rawReady(double* accel, double* upVector, double* vertAccel);

/* when ready update and get data */
void vertaccel_updateData();
double vertaccel_getValue();

/***************/
/* calibration */
/***************/

/* set the calibration vector in EEPROM */
void vertaccel_saveCalibration(double* cal);

/* give the current calibration coefficients */
double* vertaccel_getCalibration(void);

#endif
