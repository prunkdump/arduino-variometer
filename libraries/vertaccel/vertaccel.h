#ifndef VERTACCEL_H
#define VERTACCEL_H

#include <Arduino.h>
#include <LightInvensense.h>

/* eeprom calibration adresses */
#define VERTACCEL_ACCEL_EEPROM_ADDR 0x00
#define VERTACCEL_ACCEL_EEPROM_TAG 0x9907
#define VERTACCEL_MAG_EEPROM_ADDR 0x10
#define VERTACCEL_MAG_EEPROM_TAG 0x8643

/* accelerometer parameters */
#define VERTACCEL_G_TO_MS 9.80665

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

void vertaccel_readAccelCalibration(void);
void vertaccel_saveAccelCalibration(double* cal);
void vertaccel_readMagCalibration(void);
void vertaccel_saveMagCalibration(double* cal);
void vertaccel_getAccelCalibration(double* cal);
void vertaccel_getMagCalibration(double* cal);

#endif
