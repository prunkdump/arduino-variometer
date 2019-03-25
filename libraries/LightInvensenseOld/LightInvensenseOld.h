#ifndef LIGHT_INVENSENSE_OLD_H
#define LIGHT_INVENSENSE_OLD_H

#include <Arduino.h>
#include <InvenSense_defines.h>

#define LIGHT_INVENSENSE_COMPASS_ADDR (0x0C)

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

/* init */
int fastMPUInit(bool startMPU = true);

/* if needed set biases before (or after) starting DMP */
int fastMPUSetGyroBias(const unsigned char* bias);  //in machine representation
int fastMPUReadGyroBias(unsigned char* bias);
int fastMPUSetGyroBias(const int16_t* bias);
int fastMPUSetGyroBiasQ16(const int32_t* bias); //q16bias = bias << 16

int fastMPUSetAccelBias(const unsigned char* bias); //in machine representation
int fastMPUReadAccelBias(unsigned char* bias);
int fastMPUSetAccelBias(const int16_t* bias);
int fastMPUSetAccelBiasQ15(const int32_t* bias);  //q15bias = bias << 15

void fastMPUStart(void); //if not started already

/* read gyro/accel/quat measures */
int fastMPUReadFIFO(int16_t *gyro, int16_t *accel, int32_t *quat);

/* tap callback */
void fastMPUSetTapCallback(void (*callback)(unsigned char, unsigned char));

#ifdef AK89xx_SECONDARY
/* mag measures */
bool fastMPUMagReady(void);
int fastMPUReadRawMag(int16_t* mag);
int fastMPUReadMag(int16_t* mag);
unsigned char* fastMPUGetMagSensAdj(void);
#endif

/******************/
/* CUSTOM DEFINES */
/******************/

/* quat scale */
#define LIGHT_INVENSENSE_QUAT_SCALE_SHIFT 30
#define LIGHT_INVENSENSE_QUAT_SCALE ((double)(1LL << LIGHT_INVENSENSE_QUAT_SCALE_SHIFT))

/* accel scale */
#if LIGHT_INVENSENSE_ACCEL_FSR == INV_FSR_2G
#define LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT 14
#define LIGHT_INVENSENSE_ACCEL_SCALE ((double)(1LL << LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT))
#elif LIGHT_INVENSENSE_ACCEL_FSR == INV_FSR_4G
#define LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT 13
#define LIGHT_INVENSENSE_ACCEL_SCALE ((double)(1LL << LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT))
#elif LIGHT_INVENSENSE_ACCEL_FSR == INV_FSR_8G
#define LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT 12
#define LIGHT_INVENSENSE_ACCEL_SCALE ((double)(1LL << LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT))
#elif LIGHT_INVENSENSE_ACCEL_FSR == INV_FSR_16G
#define LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT 11
#define LIGHT_INVENSENSE_ACCEL_SCALE ((double)(1LL << LIGHT_INVENSENSE_ACCEL_SCALE_SHIFT))
#endif

/* mag scale */
/* !!! horizontal component (around 128) !!! */
#define LIGHT_INVENSENSE_MAG_PROJ_SCALE_SHIFT 7
#define LIGHT_INVENSENSE_MAG_PROJ_SCALE ((double)(1LL << LIGHT_INVENSENSE_MAG_SCALE_SHIFT))

#endif
