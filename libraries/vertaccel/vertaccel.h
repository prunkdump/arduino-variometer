/* vertaccel -- Compute vertical acceleration from IMU 
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

#ifndef VERTACCEL_H
#define VERTACCEL_H

#include <Arduino.h>
#include <VarioSettings.h>

//to know about the magnetometer 
#include <LightInvensense.h>

/* G to ms convertion */
#define VERTACCEL_G_TO_MS 9.80665

/* use mag sensitivity adjustement */
#define VERTACCEL_USE_MAG_SENS_ADJ

/* enable EEPROM functionnalities */
#define VERTACCEL_ENABLE_EEPROM

/*############################################*/
/* You can compile vertaccel with static      */
/* calibration coefficients.                  */
/* For this define  :                         */
/*                                            */
/* #define VERTACCEL_STATIC_CALIBRATION       */
/*                                            */
/* And set the values with :                  */
/*                                            */
/* #define VERTACCEL_GYRO_CAL_BIAS            */
/* #define VERTACCEL_ACCEL_CAL_BIAS           */
/* #define VERTACCEL_ACCEL_CAL_SCALE          */
/* #define VERTACCEL_MAG_CAL_BIAS             */
/* #define VERTACCEL_MAG_CAL_PROJ_SCALE       */
/*                                            */
/*############################################*/

#if defined(VERTACCEL_STATIC_CALIBRATION) && \
  ( !defined(VERTACCEL_GYRO_CAL_BIAS) || \
    !defined(VERTACCEL_ACCEL_CAL_BIAS) || \
    !defined(VERTACCEL_ACCEL_CAL_SCALE) || \
    (defined(AK89xx_SECONDARY) && !defined(VERTACCEL_MAG_CAL_BIAS)) || \
    (defined(AK89xx_SECONDARY) && !defined(VERTACCEL_MAG_CAL_PROJ_SCALE)) )
#error Static calibration is enabled but static values not given !
#endif

/* base struct */
struct VertaccelCalibration {
  int16_t bias[3];
  int16_t scale;
};

/* calibration settings */
struct VertaccelSettings {
  uint8_t gyroCal[12]; //stored in machine representation
  VertaccelCalibration accelCal;
#ifdef AK89xx_SECONDARY
  VertaccelCalibration magCal;
#endif
};

/* default settings */
#define VERTACCEL_DEFAULT_MAG_PROJ_COEFF 0.745346
#define VERTACCEL_DEFAULT_MAG_CAL_PROJ_SCALE -16689

/* enable static or default settings */
#ifdef VERTACCEL_STATIC_CALIBRATION

constexpr VertaccelSettings vertaccelSettings = {
  VERTACCEL_GYRO_CAL_BIAS
  ,{VERTACCEL_ACCEL_CAL_BIAS, VERTACCEL_ACCEL_CAL_SCALE}
#ifdef AK89xx_SECONDARY
  ,{VERTACCEL_MAG_CAL_BIAS, VERTACCEL_MAG_CAL_PROJ_SCALE}
#endif //AK89xx_SECONDARY
};

#else
  
constexpr VertaccelSettings defaultVertaccelSettings = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
  ,{ {0, 0, 0}, 0 }
#ifdef AK89xx_SECONDARY
  ,{ {0, 0, 0}, VERTACCEL_DEFAULT_MAG_CAL_PROJ_SCALE}
#endif //AK89xx_SECONDARY
};

#endif //VERTACCEL_STATIC_CALIBRATION


/************************/
/* calibration settings */
/************************/

/* this is default values, it can be overwrited */

/* Bias is multiplied by 2^multiplier                 */
/* maximum bias correction value is 2^(15-multiplier) */
#ifndef VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER
#define VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER 6
#endif

#ifndef VERTACCEL_MAG_CAL_BIAS_MULTIPLIER
#define VERTACCEL_MAG_CAL_BIAS_MULTIPLIER 4
#endif

/* Measure is scaled by (1 + scale/2^multiplier) */
/* minimum scale is 1 - 2^(15-mutiplier) and maximum scale 1 + 2^(15-mutiplier) */
/* !! A default scale is applied before (see LightInvensense.h) !! */
#ifndef VERTACCEL_CAL_SCALE_MULTIPLIER
#define VERTACCEL_CAL_SCALE_MULTIPLIER 16
#endif

#ifdef VERTACCEL_ENABLE_EEPROM
/* read settings from EEPROM */
// see  Vertaccel::readEEPROMSettings()
#define VERTACCEL_GYRO_CAL_EEPROM_ADDR 0x00
#define VERTACCEL_GYRO_CAL_EEPROM_TAG 0xf4e2
#define VERTACCEL_ACCEL_CAL_EEPROM_ADDR 0x0C
#define VERTACCEL_ACCEL_CAL_EEPROM_TAG 0xee54
#define VERTACCEL_MAG_CAL_EEPROM_ADDR 0x14
#define VERTACCEL_MAG_CAL_EEPROM_TAG 0x49f2
#define VERTACCEL_MAG_PROJ_EEPROM_ADDR 0x1C
#define VERTACCEL_MAG_PROJ_EEPROM_TAG 0x67fa
#endif //VERTACCEL_ENABLE_EEPROM


/*************************/
/*                       */
/*    The main class     */
/*                       */
/*************************/
class Vertaccel{
  
 public:
#ifndef VERTACCEL_STATIC_CALIBRATION
 Vertaccel(VertaccelSettings baseSettings = defaultVertaccelSettings) : settings(baseSettings) { }
#endif
  
  /* init device */
  void init(void);

  /* access to sensors */
  static uint8_t readRawAccel(int16_t* accel, int32_t* quat);
#ifdef AK89xx_SECONDARY
  static uint8_t readRawMag(int16_t* mag);
#endif //AK89xx_SECONDARY

  /* compute measures */
  void compute(int16_t *imuAccel, int32_t *imuQuat, double* vertVector, double& vertAccel);
#ifdef AK89xx_SECONDARY
  void computeNorthVector(double* vertVector, int16_t* mag, double* northVector);
#endif
  
  /* calibration class methods */
  static void readCurrentDMPGyroCalibration(unsigned char* gyroCal);

#ifdef VERTACCEL_ENABLE_EEPROM
  /* EEPROM methods */
  static VertaccelSettings readEEPROMSettings(void);
  static void saveGyroCalibration(const uint8_t* gyroCal);
  static void readGyroCalibration(uint8_t* gyroCal);
  static void saveAccelCalibration(const VertaccelCalibration& accelCal);
  static void readAccelCalibration(VertaccelCalibration& accelCal);
#ifdef AK89xx_SECONDARY
  static void saveMagCalibration(const VertaccelCalibration& magCal);
  static void readMagCalibration(VertaccelCalibration& magCal);
#endif //AK89xx_SECONDARY
#endif //VERTACCEL_ENABLE_EEPROM
 
  
 private:
#ifdef VERTACCEL_STATIC_CALIBRATION
  static constexpr VertaccelSettings settings = vertaccelSettings;
  static constexpr uint8_t gyroCalArray[12] = VERTACCEL_GYRO_CAL_BIAS; //need to be passed as pointer
  static constexpr int32_t accelCalArray[3] = { (int32_t)vertaccelSettings.accelCal.bias[0] * ((int32_t)1 << (15 - VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER)),
						(int32_t)vertaccelSettings.accelCal.bias[1] * ((int32_t)1 << (15 - VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER)),
						(int32_t)vertaccelSettings.accelCal.bias[2] * ((int32_t)1 << (15 - VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER)) }; //passed as pointer
#else
  VertaccelSettings settings;
#endif 
};


#endif
