#ifndef MS5611_H
#define MS5611_H

#include <Arduino.h>
#include <I2Cdev.h>

/*###############################################################*/
/*                     !!!!!!!!!!!!!!!                           */
/*                     !!! WARNING !!!                           */
/*                     !!!!!!!!!!!!!!!                           */
/* The ms5611 altimeter is very sensitive to measure frequency.  */
/* So this library use TIMER2 interrupts to make I2C requests    */
/* at very stable frequency.                                     */
/* Therefore :                                                   */
/* 1) you can't use timer2 while using this library              */
/* 2) To use I2C with other device you need to link to a special */
/*    I2CDev library provided as I2CDev_ms5611. Try to make the  */
/*    I2C request as short as possible.                          */
/* 3) To get the higtest precision, init fastwire at 400Htz with */
/*    Fastwire::setup(400,0) before initializing the device.     */
/* From your point of view, you DON'T need to get the ms5611     */
/* data at stable frequency. The timer's interrupts update the   */
/* global variables for you. You can get the values when you     */ 
/* want.                                                         */ 
/*###############################################################*/

/* the pressure normalized sea level pressure */  
#define MS5611_BASE_SEA_PRESSURE 1013.25

#define MS5611_ADDRESS (0x77)
#define MS5611_CMD_RESET (0x1E)
#define MS5611_CMD_READ_PROM (0xA2)
#define MS5611_CMD_CONV_D1 (0x46)
#define MS5611_CMD_CONV_D2 (0x58)
#define MS5611_CMD_ADC_READ (0x00)

#define MS5611_RESET_DELAY 3
#define MS5611_CONV_DELAY 9

#define MS5611_STEP_READ_TEMP 0
#define MS5611_STEP_READ_PRESSURE 1

/* the measure period need to be greater than 8.22 ms */
/* the library use a 1024 prescale so the time unit is 1024/F_CPU */
/* the INTERRUPT_COMPARE can't be greater than 255 */
/* but a greater value give less code interrupts */
/* the final period is 1024/F_CPU * INTERRUPT_COMPARE */
/* in seconds */
#define MS5611_INTERRUPT_COMPARE 468
#define MS5611_INTERRUPT_START_DELAY 1000

/* ################################ */

/* first init the altimeter */
void ms5611_init();

/* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
/* once the variometer is initialized */
/* you can't use i2c anymore without  */
/* locking the variometer             */
/* !! USE THE MS5611 i2cdev LIBRARY!! */
/* !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! */
void ms5611_lock();
void ms5611_release();

/* check if you have new data */
boolean ms5611_dataReady(void);

/* then compute compensated temperature, pressure and alti values */
void ms5611_updateData(void);

/* and finally get computed values */
double ms5611_getTemperature();
double ms5611_getPressure();
double ms5611_getAltitude();

#endif
