#ifndef VERTACCEL_H
#define VERTACCEL_H

#include <Arduino.h>
#include <LightInvensense.h>

/* use mag sensitivity adjustement */
#define VERTACCEL_USE_MAG_SENS_ADJ

/* G to ms convertion */
#define VERTACCEL_G_TO_MS 9.80665

/* enable EEPROM functionnalities */
#define VERTACCEL_ENABLE_EEPROM


/************************/
/* calibration settings */
/************************/

/* Bias is multiplied by 2^multiplier                 */
/* maximum bias correction value is 2^(15-multiplier) */
#define VERTACCEL_ACCEL_CAL_BIAS_MULTIPLIER 6
#define VERTACCEL_MAG_CAL_BIAS_MULTIPLIER 4

/* Measure is scaled by (1 + scale/2^multiplier) */
/* minimum scale is 1 - 2^(15-mutiplier) and maximum scale 1 + 2^(15-mutiplier) */
/* !! A default scale is applied before (see LightInvensense.h) !! */ 
#define VERTACCEL_CAL_SCALE_MULTIPLIER 16

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

const VertaccelSettings defaultVertaccelSettings = {
  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
  ,{ {0, 0, 0}, 0 }
#ifdef AK89xx_SECONDARY
  ,{ {0, 0, 0}, VERTACCEL_DEFAULT_MAG_CAL_PROJ_SCALE}
#endif
};

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


/******************/
/* the main class */
/******************/

/* flags returned by all the ready/read functions */
#define VERTACCEL_HAVE_ACCEL 1
#define VERTACCEL_HAVE_MAG 2

class Vertaccel{
  
 public:
 Vertaccel(const VertaccelSettings& settings = defaultVertaccelSettings) : settings(settings) { }

  /* init device */
  void init(void);

  /* !!! WARNING : run as often as possible to check the FIFO stack !!! */
  uint8_t dataReady(void);
  uint8_t dataFullReady(double* accel, double* quat, double* v, double* n);

  /* when ready get data */
  double getValue(void);
#ifdef AK89xx_SECONDARY
  double* getNorthVector(void);
#endif //AK89xx_SECONDARY

  /* direct access to sensors */
  static uint8_t readRawAccel(int16_t* accel, int32_t* quat);
#ifdef AK89xx_SECONDARY
  static uint8_t readRawMag(int16_t* mag);
#endif //AK89xx_SECONDARY

  /* calibration methods */
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
  const VertaccelSettings& settings;
  double vertAccel;
#ifdef AK89xx_SECONDARY
  double northVector[2];
#endif //AK89xx_SECONDARY
};


#endif
