/* TwoWireScheduler -- Interrupt driven Two Wire devices scheduler
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

#ifndef TWO_WIRE_SCHEDULER_H
#define TWO_WIRE_SCHEDULER_H

#include <Arduino.h>
#include <VarioSettings.h>

#ifdef HAVE_BMP280
#include <bmp280.h>
#else
#include <ms5611.h>
#endif

#ifdef HAVE_ACCELEROMETER
#include <vertaccel.h>
#endif

/* The unit is 0.008 ms    */
/* 128 prescale on 16Mhz   */
/* 64 prescale on 8 Mhz    */
#if F_CPU >= 16000000L
#define TWO_WIRE_SCHEDULER_INTERRUPT_PRESCALE 0b00000101
#else
#define TWO_WIRE_SCHEDULER_INTERRUPT_PRESCALE 0b00000100
#endif //F_CPU

/******************************************/
/* The barometer is the reference freq    */
/* read take max 0.3 ms                   */
/*                                        */
/* bmp280                                 */ 
/* -> max measurement time : 43.2 ms      */
/* -> 43.5 ms period ( approx 20 Hz )     */
/* -> ref period = 40 ( more than 20 Hz ) */
/* -> compare = 136                       */
/* -> real unit = 1.088 ms                */
/*                                        */
/* ms5611                                 */
/* -> max measurement time : 9.04 ms      */
/* -> 9.34 ms period ( approx 100 Hz )    */
/* -> ref period = 8 ( more than 100 Hz ) */
/* -> compare = 146                       */
/* -> real unit = 1.168 ms                */
/*                                        */
/* mpu fifo                               */
/* -> freq = 100 Hz but checked at 200 Hz */
/* -> period = 4 ( more than 200 Hz )     */
/*                                        */
/* magnetometer                           */
/* -> freq = 10 Hz but checked at 20 Hz   */
/* -> period = 40 ( more than 20 Hz )     */
/******************************************/
#ifdef HAVE_BMP280
#define TWO_WIRE_SCHEDULER_INTERRUPT_COMPARE 136
#else
#define TWO_WIRE_SCHEDULER_INTERRUPT_COMPARE 146
#endif

/* The scheduler                                          */
/* see how TW request are shifted                         */
/* so they don't interact each other                      */
/* !! mpu fifo interrupt take 1.4 ms, more than 1 unit !! */ 
#define TWO_WIRE_SCHEDULER_MS5611_PERIOD 8
#define TWO_WIRE_SCHEDULER_MS5611_SHIFT 0
#define TWO_WIRE_SCHEDULER_BMP280_PERIOD 40
#define TWO_WIRE_SCHEDULER_BMP280_SHIFT 0
#define TWO_WIRE_SCHEDULER_IMU_PERIOD 4
#define TWO_WIRE_SCHEDULER_IMU_SHIFT 1
#define TWO_WIRE_SCHEDULER_MAG_PERIOD 40
#define TWO_WIRE_SCHEDULER_MAG_SHIFT 3


/*******************************************************/
/* !!!                                             !!! */
/* To use TWScheduler, init static variables :         */
/*                                                     */
/* Ms5611 TWScheduler::ms5611;                         */
/* Vertaccel TWScheduler::vertaccel;                   */
/* !!!                                             !!! */
/*******************************************************/
class TWScheduler {

 public:
  /* static class devices to define */
#ifdef HAVE_BMP280
  static Bmp280 bmp280;
#else  
  static Ms5611 ms5611;
#endif
  
#ifdef HAVE_ACCELEROMETER
  static Vertaccel vertaccel;
#endif

  /* init both devices but not the TW bus */
  static void init(void);

  /* barometer part */
  static bool havePressure(void);
  static double getAlti(void);

#ifdef HAVE_ACCELEROMETER
  /* IMU part */
  /* tap callback is triggered by getRawAccel and getAccel */
  static bool haveAccel(void);
  static void getRawAccel(int16_t* rawAccel, int32_t* quat);
  static double getAccel(double* vertVector); //vertVector = NULL if not needed
#ifdef AK89xx_SECONDARY
  static bool haveMag(void);
  static void getRawMag(int16_t* rawMag);
  static void getNorthVector(double* vertVector, double* northVector); //give the vertVector obtained previously
#endif //AK89xx_SECONDARY
#endif //HAVE_ACCELEROMETER
  
  /* the main interrupt */
  static void mainInterrupt(void);

 private:
  static uint8_t volatile status;
#ifdef HAVE_BMP280 
  static uint8_t volatile bmp280Output[2*3];  //two bmp280 output measures
  static uint8_t volatile bmp280Count;
#else
  static int8_t volatile ms5611Step; 
  static uint8_t volatile ms5611Output[3*3];  //three ms5611 output measures
  static uint8_t volatile ms5611Count;
#endif
#ifdef HAVE_ACCELEROMETER
  static uint8_t volatile checkOutput[2];
  static uint8_t volatile imuOutput[LIGHT_INVENSENSE_COMPRESSED_DMP_PAQUET_LENGTH]; //imu dmp fifo output
  static uint8_t volatile imuCount;
#ifdef AK89xx_SECONDARY
  static uint8_t volatile magOutput[8];    //magnetometer output
  static uint8_t volatile magCount;
#endif //AK89xx_SECONDARY
#endif //HAVE_ACCELEROMETER
  
  /* private interrupt methods */
#ifdef HAVE_BMP280
  static void bmp280Interrupt(void);
  static void bmp280OutputCallback(void);
#else
  static void ms5611Interrupt(void);
  static void ms5611TempCallback(void);
  static void ms5611OutputCallback(void);
#endif
#ifdef HAVE_ACCELEROMETER
  static void imuInterrupt(void);
  static void imuCheckFifoCountCallBack(void);
  static void imuHaveFifoDataCallback(void);
#ifdef AK89xx_SECONDARY
  static void magInterrupt(void);
  static void magCheckStatusCallback(void);
  static void magHaveDataCallback(void);
#endif //AK89xx_SECONDARY
#endif //HAVE_ACCELEROMETER
};

extern TWScheduler twScheduler;

#endif //TWO_WIRE_SCHEDULER_H
