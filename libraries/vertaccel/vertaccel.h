#ifndef VERTACCEL_H
#define VERTACCEL_H

#include <Arduino.h>
#include <LightInvensense.h>

/* eeprom calibration adresses */
#define VERTACCEL_EPROM_TAG 9806
#define VERTACCEL_EPROM_ADDR 0x00

/* accelerometer parameters */
#define VERTACCEL_G_TO_MS 9.80665

#define CALIBRATION_MPU_TIMEOUT 1000
#define CALIBRATION_ZACCEL_THRESHOLD -0.95

/* calibration condition based on accelerometer */
/* !! you need to init vertaccel first !! */
boolean MpuCalibrationCond(void);

/*******************/
/* init the device */
/*******************/
void vertaccel_init(void);

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

boolean vertaccel_readAvailableCalibration(void);

#endif
