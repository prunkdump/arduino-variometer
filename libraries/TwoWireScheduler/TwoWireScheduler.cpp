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

#include <TwoWireScheduler.h>

#include <Arduino.h>

#include <IntTW.h>
#include <avr/pgmspace.h>

#ifdef HAVE_BMP280
#include <bmp280.h>
#else
#include <ms5611.h>
#endif

#include <VarioSettings.h>

#ifdef HAVE_ACCELEROMETER
#include <LightInvensense.h>
#include <vertaccel.h>
#endif

#define TEMP_READ 0
#define PRESSURE_RELEASED 1
#define HAVE_PRESSURE 2
#define ACCEL_RELEASED 3
#define HAVE_ACCEL 4
#define MAG_RELEASED 5
#define HAVE_MAG 6

#define bset(bit) status |= (1 << bit)
#define bunset(bit) status &= ~(1 << bit)
#define bisset(bit) (status & (1 << bit))

TWScheduler twScheduler;

/*********************/
/* static class data */
/*********************/
uint8_t volatile TWScheduler::status = 0;   //no problem to not release at start as there is no values
#ifdef HAVE_BMP280 
uint8_t volatile TWScheduler::bmp280Output[2*3];  //two bmp280 output measures
uint8_t volatile TWScheduler::bmp280Count = TWO_WIRE_SCHEDULER_BMP280_SHIFT;
#else
int8_t volatile TWScheduler::ms5611Step = 0; 
uint8_t volatile TWScheduler::ms5611Output[3*3];  //three ms5611 output measures
uint8_t volatile TWScheduler::ms5611Count = TWO_WIRE_SCHEDULER_MS5611_SHIFT;
#endif
#ifdef HAVE_ACCELEROMETER
uint8_t volatile TWScheduler::checkOutput[2];
uint8_t volatile TWScheduler::imuOutput[LIGHT_INVENSENSE_COMPRESSED_DMP_PAQUET_LENGTH]; //imu dmp fifo output
uint8_t volatile TWScheduler::imuCount = TWO_WIRE_SCHEDULER_IMU_SHIFT;
#ifdef AK89xx_SECONDARY
uint8_t volatile TWScheduler::magOutput[8];    //magnetometer output
uint8_t volatile TWScheduler::magCount = TWO_WIRE_SCHEDULER_MAG_SHIFT;
#endif //AK89xx_SECONDARY
#endif //HAVE_ACCELEROMETER


#ifdef HAVE_BMP280
/***************/
/* bmp280 part */
/***************/

static const uint8_t bmp280Step[] PROGMEM = { INTTW_ACTION(BMP280_STATIC_ADDRESS, INTTW_WRITE),
					      INTTW_DEST(1, INTTW_IN_CMD),
					      BMP280_PRESS_REG,
					      INTTW_ACTION(BMP280_STATIC_ADDRESS, INTTW_READ),
					      INTTW_DEST(6, INTTW_AT_POINTER),
					      INTTW_ACTION(BMP280_STATIC_ADDRESS, INTTW_WRITE),
					      INTTW_DEST(2, INTTW_IN_CMD),
					      BMP280_CTRL_MEAS_REG,
					      BMP280_MEASURE_CONFIG };

static void TWScheduler::bmp280Interrupt(void) {

  /* read previous measure and launch next */
  /* we need to lock */
  intTW.setRxBuffer(bmp280Output);
  bunset(PRESSURE_RELEASED);
  intTW.start(bmp280Step, sizeof(bmp280Step), INTTW_USE_PROGMEM, bmp280OutputCallback);
}


static void TWScheduler::bmp280OutputCallback(void) {

  /* done ! */
  status |= (1 << HAVE_PRESSURE) | (1 << PRESSURE_RELEASED);
}


static bool TWScheduler::havePressure(void) {

  return bisset(HAVE_PRESSURE);
}


static double TWScheduler::getAlti(void) {

  /* wait realease */
  while( ! bisset(PRESSURE_RELEASED) ) { }

  /* copy needed values */
  uint8_t bmp280Values[6];
  
  noInterrupts();
  for(int i = 0; i<6; i++) {
    bmp280Values[i] =  bmp280Output[i];
  }
  bunset(HAVE_PRESSURE);
  interrupts();

  /* compute pressure and temp */
  double temperature, pressure;
  bmp280.computeMeasures(&bmp280Values[0], &bmp280Values[3], temperature, pressure);

  /* get corresponding alti */
  double alti = bmp280.computeAltitude(pressure);

  return alti;
}

#else //! HAVE_BMP280
/***************/
/* ms5611 part */
/***************/

static const uint8_t ms5611Step1[] PROGMEM = { INTTW_ACTION(MS5611_STATIC_ADDRESS, INTTW_WRITE),
					       INTTW_DEST(1, INTTW_IN_CMD),
					       MS5611_CMD_ADC_READ,
					       INTTW_ACTION(MS5611_STATIC_ADDRESS, INTTW_READ),
					       INTTW_DEST(3, INTTW_AT_POINTER),
					       INTTW_ACTION(MS5611_STATIC_ADDRESS, INTTW_WRITE),
					       INTTW_DEST(1, INTTW_IN_CMD),
					       MS5611_CMD_CONV_D2 };

static const uint8_t ms5611Step2[] PROGMEM = { INTTW_ACTION(MS5611_STATIC_ADDRESS, INTTW_WRITE),
					       INTTW_DEST(1, INTTW_IN_CMD),
					       MS5611_CMD_ADC_READ,
					       INTTW_ACTION(MS5611_STATIC_ADDRESS, INTTW_READ),
					       INTTW_DEST(3, INTTW_AT_POINTER),
					       INTTW_ACTION(MS5611_STATIC_ADDRESS, INTTW_WRITE),
					       INTTW_DEST(1, INTTW_IN_CMD),
					       MS5611_CMD_CONV_D1 };

static void TWScheduler::ms5611Interrupt(void) {

  if( ms5611Step == 0 ) {

    bunset(TEMP_READ);
    intTW.setRxBuffer(ms5611Output);
    intTW.start(ms5611Step1, sizeof(ms5611Step1), INTTW_USE_PROGMEM, ms5611TempCallback);
    ms5611Step = 1;
  }

  else {

    /* if can't get temp, don't go further */
    if( ! bisset(TEMP_READ) ) {
      return;
    }

    /* copy the first value to get it at any time from main loop */
    for(int i = 0; i<3; i++) {
      ms5611Output[i+3] = ms5611Output[i];
    }

    /* get the next value */
    /* we need to lock */
    intTW.setRxBuffer(&ms5611Output[6]);
    bunset(PRESSURE_RELEASED);
    intTW.start(ms5611Step2, sizeof(ms5611Step2), INTTW_USE_PROGMEM, ms5611OutputCallback);
    ms5611Step = 0;
  }
}

static void TWScheduler::ms5611TempCallback(void) {

  bset(TEMP_READ);
}

static void TWScheduler::ms5611OutputCallback(void) {

  /* done ! */
  status |= (1 << HAVE_PRESSURE) | (1 << PRESSURE_RELEASED);
}

static bool TWScheduler::havePressure(void) {

  return bisset(HAVE_PRESSURE);
}

static double TWScheduler::getAlti(void) {

  /* wait realease */
  while( ! bisset(PRESSURE_RELEASED) ) { }

  /* copy needed values */
  uint8_t ms5611Values[6];
  
  noInterrupts();
  for(int i = 0; i<6; i++) {
    ms5611Values[i] =  ms5611Output[i+3];
  }
  bunset(HAVE_PRESSURE);
  interrupts();

  /* compute pressure and temp */
  double temperature, pressure;
  ms5611.computeMeasures(&ms5611Values[0], &ms5611Values[3], temperature, pressure);

  /* get corresponding alti */
  double alti = ms5611.computeAltitude(pressure);

  return alti;
}
#endif

#ifdef HAVE_ACCELEROMETER
/************/
/* IMU part */
/************/

static const uint8_t imuReadFifoCount[] PROGMEM = { INTTW_ACTION(INV_HW_ADDR, INTTW_WRITE),
						    INTTW_DEST(1, INTTW_IN_CMD),
						    INV_REG_FIFO_COUNT_H,
						    INTTW_ACTION(INV_HW_ADDR, INTTW_READ),
						    INTTW_DEST(2, INTTW_AT_POINTER) };


static const uint8_t imuReadFifo[] PROGMEM = { INTTW_ACTION(INV_HW_ADDR, INTTW_WRITE),
					       INTTW_DEST(1, INTTW_IN_CMD),
					       INV_REG_FIFO_R_W,
					       INTTW_ACTION(INV_HW_ADDR, INTTW_READ),
					       INTTW_DEST(LIGHT_INVENSENSE_COMPRESSED_DMP_PAQUET_LENGTH, INTTW_AT_POINTER) };


static void TWScheduler::imuInterrupt(void) {

  /* check FiFo for available measures */
  intTW.setRxBuffer(checkOutput);
  intTW.start(imuReadFifoCount, sizeof(imuReadFifoCount), INTTW_USE_PROGMEM | INTTW_KEEP_BUS, imuCheckFifoCountCallBack);
}


static void TWScheduler::imuCheckFifoCountCallBack(void) {

  /* we have FiFo count */
  uint16_t fifoCount = (((uint16_t)checkOutput[0]) << 8) | checkOutput[1];

  /* launch FiFo read if OK */
  int8_t fifoState = fastMPUHaveFIFOPaquet(fifoCount);
  if( fifoState > 0 ) {

    /* we need to lock */
    intTW.setRxBuffer(imuOutput);
    bunset(ACCEL_RELEASED);
    intTW.start(imuReadFifo, sizeof(imuReadFifo), INTTW_USE_PROGMEM, imuHaveFifoDataCallback);
  } else {

    /* else stop TW communication */
    intTW.stop();
  }
}

static void TWScheduler::imuHaveFifoDataCallback(void) {

  /* done ! */
  status |= (1 << HAVE_ACCEL) | (1 << ACCEL_RELEASED);
}

bool TWScheduler::haveAccel(void) {

  return bisset(HAVE_ACCEL);
}

void TWScheduler::getRawAccel(int16_t* rawAccel, int32_t* quat) {
  
  /***************/
  /* check accel */
  /***************/

  /* wait realease */
  while( ! bisset(ACCEL_RELEASED) ) { }
  
  /* first copy fifo packet */
  uint8_t fifoPacket[LIGHT_INVENSENSE_COMPRESSED_DMP_PAQUET_LENGTH];
  noInterrupts();
  for(int i = 0; i<LIGHT_INVENSENSE_COMPRESSED_DMP_PAQUET_LENGTH; i++) {
    fifoPacket[i] =  imuOutput[i];
  }
  bunset(HAVE_ACCEL);
  interrupts();

  /* parse FiFo packet to get raw measures */
  uint8_t tap;
  fastMPUParseFIFO(fifoPacket, NULL, rawAccel, quat, tap);

  /* check tap : use callback if needed */
  fastMPUCheckTap(tap);
}
  

double TWScheduler::getAccel(double* vertVector) {

  /*****************/
  /* get raw accel */
  /*****************/
  int16_t rawAccel[3];
  int32_t quat[4];
  
  getRawAccel(rawAccel, quat);
  
  /* compute vertVector and vertAccel */
  double vertAccel;
  if( vertVector ) {
    vertaccel.compute(rawAccel, quat, vertVector, vertAccel);
  } else {
    double tmpVertVector[3];
    vertaccel.compute(rawAccel, quat, tmpVertVector, vertAccel);
  }

  /* done */
  return vertAccel;
}


#ifdef AK89xx_SECONDARY
/************/
/* Mag part */
/************/

static const uint8_t magReadStatus[] PROGMEM = { INTTW_ACTION(INV_HW_ADDR, INTTW_WRITE),
						 INTTW_DEST(1, INTTW_IN_CMD),
						 INV_REG_I2C_MST_STATUS,
						 INTTW_ACTION(INV_HW_ADDR, INTTW_READ),
						 INTTW_DEST(1, INTTW_AT_POINTER) };

static const uint8_t magReadData[] PROGMEM = { INTTW_ACTION(INV_HW_ADDR, INTTW_WRITE),
					       INTTW_DEST(1, INTTW_IN_CMD),
					       INV_REG_RAW_COMPASS,
					       INTTW_ACTION(INV_HW_ADDR, INTTW_READ),
					       INTTW_DEST(8, INTTW_AT_POINTER) };

static void TWScheduler::magInterrupt(void) {

  /* check for available measures */
  intTW.setRxBuffer(checkOutput);
  intTW.start(magReadStatus, sizeof(magReadStatus), INTTW_USE_PROGMEM | INTTW_KEEP_BUS, magCheckStatusCallback);

}

static void TWScheduler::magCheckStatusCallback(void) {

  /* check if new measure */
  if( checkOutput[0] & 0x40 ) {

    /* read measure */
    /* we need to lock */
    intTW.setRxBuffer(magOutput);
    bunset(MAG_RELEASED);
    intTW.start(magReadData, sizeof(magReadData), INTTW_USE_PROGMEM, magHaveDataCallback);
  } else {

    /* stop TW communication */
    intTW.stop();
  }
}

static void TWScheduler::magHaveDataCallback(void) {

  /* done ! */
  status |= (1 << HAVE_MAG) | (1 << MAG_RELEASED);
}

bool TWScheduler::haveMag(void) {

  return bisset(HAVE_MAG);
}

static void TWScheduler::getRawMag(int16_t* rawMag) {
  
  /*************/
  /* check mag */
  /*************/
  if( bisset(HAVE_MAG) ) {

    /* wait realease */
    while( ! bisset(MAG_RELEASED) ) { }

    /* copy mag data */
    uint8_t magData[8];
    noInterrupts();
    for(int i = 0; i<8; i++) {
      magData[i] =  magOutput[i];
    }
    bunset(HAVE_MAG);
    interrupts();

    /* parse mag data */
#ifdef VERTACCEL_USE_MAG_SENS_ADJ    
    if( fastMPUParseMag(magData, rawMag) >= 0 ) {
#else
    if( fastMPUParseRawMag(magData, rawMag) >= 0 ) {
#endif
    }
  }
}

static void TWScheduler::getNorthVector(double* vertVector, double* northVector) {

  /* get raw mag */
  int16_t rawMag[3];
  getRawMag(rawMag);

  /* compute north vector */
  vertaccel.computeNorthVector(vertVector, rawMag, northVector);
}

#endif //AK89xx_SECONDARY
#endif //HAVE_ACCELEROMETER

/*---------------------*/
/*                     */
/*      Scheduler      */
/*                     */
/*---------------------*/
static void TWScheduler::init(void) {

  /* init the devices */
#ifdef HAVE_BMP280
  bmp280.init();
#else
  ms5611.init();
#endif
#ifdef HAVE_ACCELEROMETER
  vertaccel.init();
#endif

  /* launch timer */
  noInterrupts();   // disable all interrupts

#ifdef TIMER2_COMPA_vect
  TCCR2A = 0b00000010; //CTC MODE
  TCCR2B = TWO_WIRE_SCHEDULER_INTERRUPT_PRESCALE;
  TIMSK2 = 0b00000010; //enable CompA
  
  TCNT2  = 0;
  OCR2A = TWO_WIRE_SCHEDULER_INTERRUPT_COMPARE;
#else
  TCCR3A = 0b00000000; //CTC MODE
  TCCR3B = 0b00001011; //64 prescale
  TIMSK3 = 0b00000010; //enable CompA
  
  TCNT3  = 0;
  OCR3A = 2*TWO_WIRE_SCHEDULER_INTERRUPT_COMPARE; //we assume 16Mhz
#endif
  
  interrupts();
}

static void TWScheduler::mainInterrupt(void) {

  /* launch interrupts */
#ifdef HAVE_BMP280
  if( bmp280Count == 0 ) {
    bmp280Interrupt();
    bmp280Count = TWO_WIRE_SCHEDULER_BMP280_PERIOD;
  }
#else
  if( ms5611Count == 0 ) {
    ms5611Interrupt();
    ms5611Count = TWO_WIRE_SCHEDULER_MS5611_PERIOD;
  }
#endif
#ifdef HAVE_ACCELEROMETER
  if( imuCount == 0 ) {
    imuInterrupt();
    imuCount = TWO_WIRE_SCHEDULER_IMU_PERIOD;
  }
#ifdef AK89xx_SECONDARY
  if( magCount == 0 ) {
    magInterrupt();
    magCount = TWO_WIRE_SCHEDULER_MAG_PERIOD;
  }
#endif //AK89xx_SECONDARY
#endif //HAVE_ACCELEROMETER

  
  /* decrease counters */
#ifdef HAVE_BMP280
  bmp280Count--;
#else
  ms5611Count--;
#endif
#ifdef HAVE_ACCELEROMETER
  imuCount--;
#ifdef AK89xx_SECONDARY
  magCount--;
#endif //AK89xx_SECONDARY
#endif //HAVE_ACCELEROMETER
  
}


#ifdef TIMER2_COMPA_vect
ISR(TIMER2_COMPA_vect) {
#else
ISR(TIMER3_COMPA_vect) {
#endif

  TWScheduler::mainInterrupt();
}
