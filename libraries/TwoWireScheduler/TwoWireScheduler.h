#ifndef TWO_WIRE_SCHEDULER_H
#define TWO_WIRE_SCHEDULER_H

#include <Arduino.h>
#include <VarioSettings.h>

#include <ms5611.h>
#ifdef HAVE_ACCELEROMETER
#include <vertaccel.h>
#endif

/* The unit is 1.248 ms    */
/* 128 prescale on 16Mhz   */
/* 64 prescale on 8 Mhz    */
#if F_CPU >= 16000000L
#define TWO_WIRE_SCHEDULER_INTERRUPT_PRESCALE 0b00000101
#else
#define TWO_WIRE_SCHEDULER_INTERRUPT_PRESCALE 0b00000100
#endif
#define TWO_WIRE_SCHEDULER_INTERRUPT_COMPARE 156

/* The scheduler */
/* see how TW request are shifted    */
/* so they don't interact each other */
#define TWO_WIRE_SCHEDULER_MS5611_PERIOD 8
#define TWO_WIRE_SCHEDULER_MS5611_SHIFT 0
#define TWO_WIRE_SCHEDULER_IMU_PERIOD 4
#define TWO_WIRE_SCHEDULER_IMU_SHIFT 2
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
  static Ms5611 ms5611;
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
  static void getNorthVector(double* vertVector, double* northVector); //give the vertVector obtained previously
#endif //AK89xx_SECONDARY
#endif //HAVE_ACCELEROMETER
  
  /* the main interrupt */
  static void mainInterrupt(void);

 private:
  static uint8_t volatile status;
  static int8_t volatile ms5611Step; 
  static uint8_t volatile ms5611Output[3*3];  //three ms5611 output measures
  static uint8_t volatile ms5611Count;
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
  static void ms5611Interrupt(void);
  static void ms5611OutputCallback(void);
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

#endif //TWO_WIRE_SCHEDULER_H
