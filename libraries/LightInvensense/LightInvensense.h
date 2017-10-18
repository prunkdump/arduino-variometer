#ifndef LIGHT_INVENSENSE_H
#define LIGHT_INVENSENSE_H

#include <InvenSense_defines.h>

/*****************************************************************/
/*                   !!!  WARNING !!!                            */
/* This Invensense library optimize space by saving config       */
/* directly in the DMP firmware.                                 */
/*                                                               */
/* To use it :                                                   */ 
/*    1) Set the target config below (see InvenSense_defines.h)  */  
/*    2) Uncomment LIGHT_INVENSENSE_BUILD                        */
/*    3) Run createCompressedFirmware() from a sketch            */
/*       and paste the result in LightInvensense.cpp             */
/*    4) Comment LIGHT_INVENSENSE_BUILD                          */
/*    5) !!! You can't change config now without redoing !!!     */
/*       !!! there steps.                                !!!     */
/*****************************************************************/

//#define LIGHT_INVENSENSE_BUILD

/*****************/
/* TARGET CONFIG */
/*****************/

/* See InvenSense_defines.h */
#define LIGHT_INVENSENSE_GYRO_FSR INV_FSR_2000DPS
#define LIGHT_INVENSENSE_ACCEL_FSR INV_FSR_4G
#define LIGHT_INVENSENSE_DMP_SAMPLE_RATE 100
#define LIGHT_INVENSENSE_COMPASS_SAMPLE_RATE 10

/* See inv_mpu_dmp_motion_driver.h */
#ifdef LIGHT_INVENSENSE_BUILD
#include <inv_mpu_dmp_motion_driver.h>
#define LIGHT_INVENSENSE_DMP_FEATURES (DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_GYRO_CAL|DMP_FEATURE_TAP)
#endif

/* Tap settings if enabled */
#define LIGHT_INVENSENSE_TAP_AXES TAP_XYZ
#define LIGHT_INVENSENSE_TAP_THRESH 300
#define LIGHT_INVENSENSE_TAP_COUNT 1
#define LIGHT_INVENSENSE_TAP_TIME 100
#define LIGHT_INVENSENSE_TAP_TIME_MULTI 500


/*********************/
/* BUILDING FIRMWARE */
/*********************/
#ifdef LIGHT_INVENSENSE_BUILD

/* paste the output in LightInvensense.cpp */
int createCompressedFirmware(void);

#endif


/*********************************/
/* INIT MPU WITH CUSTOM FIRMWARE */
/*********************************/

int fastMPUInit(void);
int fastMPUReadFIFO(short *gyro, short *accel, long *quat);
void fastMPUSetTapCallback(void (*callback)(unsigned char, unsigned char));


/******************/
/* CUSTOM DEFINES */
/******************/

/* quat scale */
#define LIGHT_INVENSENSE_QUAT_SCALE (1073741824.0)

/* accel scale */
#if LIGHT_INVENSENSE_ACCEL_FSR == INV_FSR_2G 
#define LIGHT_INVENSENSE_ACCEL_SCALE (16384.0)
#elif LIGHT_INVENSENSE_ACCEL_FSR == INV_FSR_4G 
#define LIGHT_INVENSENSE_ACCEL_SCALE (8192.0)
#elif LIGHT_INVENSENSE_ACCEL_FSR == INV_FSR_8G
#define LIGHT_INVENSENSE_ACCEL_SCALE (4096.0)
#elif LIGHT_INVENSENSE_ACCEL_FSR == INV_FSR_16G
#define LIGHT_INVENSENSE_ACCEL_SCALE (2048.0)
#endif
  

#endif
