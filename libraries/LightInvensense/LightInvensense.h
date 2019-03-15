/* LightInvensense -- Light and optimized Invensense IMU driver
 *
 * Copyright 2016-2019 Baptiste PELLEGRIN
 * 
 * This file is part of GNUVario.
 *
 * GNUVario is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNUVario is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef LIGHT_INVENSENSE_H
#define LIGHT_INVENSENSE_H

#include <Arduino.h>
#include <InvenSense_defines.h>

#define LIGHT_INVENSENSE_COMPASS_ADDR (0x0C)

/* !!! value need to be copied manually from LightInvensence.cpp when building DMP firmware !!! */
#define LIGHT_INVENSENSE_COMPRESSED_DMP_PAQUET_LENGTH 26

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

/* to compute with you own values */
uint8_t fastMPUGetFIFOPaquetLength(void);
int8_t fastMPUHaveFIFOPaquet(uint16_t fifoCount);
void fastMPUParseFIFO(uint8_t* dmpPaquet, int16_t *gyro, int16_t *accel, int32_t *quat, uint8_t& tap);


/* tap callback */
void fastMPUSetTapCallback(void (*callback)(unsigned char, unsigned char));

/* callback is called with fastMPUReadFIFO */
/* if not used, call it manually */
void fastMPUCheckTap(uint8_t tap);


#ifdef AK89xx_SECONDARY
/* mag measures */
bool fastMPUMagReady(void);
int fastMPUReadRawMag(int16_t* mag);
int fastMPUReadMag(int16_t* mag);
unsigned char* fastMPUGetMagSensAdj(void);
void fastMPUAdjMag(int16_t* mag);

/* to use with your own values */
int fastMPUParseRawMag(uint8_t* magData, int16_t* mag);
int fastMPUParseMag(uint8_t* magData, int16_t* mag);

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
