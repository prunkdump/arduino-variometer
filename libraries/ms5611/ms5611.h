#ifndef MS5611_H
#define MS5611_H

#include <Arduino.h>
#include <VarioSettings.h>

/* the normalized sea level pressure */  
#define MS5611_BASE_SEA_PRESSURE 1013.25


/*############################################*/
/* You can compile ms5611 with static address */
/* or static calibration coefficients.        */
/* For this define the values with :          */
/*                                            */
/* #define MS5611_STATIC_ADDRESS              */
/* or                                         */
/* #define MS5611_STATIC_CALIBRATION          */
/*                                            */
/*############################################*/

static constexpr uint16_t ms5611DefaultAdress = 0x77;

struct Ms5611Calibration {

  uint16_t coeffs[6];
};


/*####################################*/
/* Here the ms5611 hardware constants */
/*####################################*/

#define MS5611_CMD_RESET (0x1E)
#define MS5611_CMD_READ_PROM (0xA2)
#define MS5611_CMD_CONV_D1 (0x46)
#define MS5611_CMD_CONV_D2 (0x58)
#define MS5611_CMD_ADC_READ (0x00)

#define MS5611_RESET_DELAY 3
#define MS5611_CONV_DELAY 9


/*-----------------------*/
/*                       */
/*     The main class    */
/*                       */
/*-----------------------*/
class Ms5611 {

 public:
#ifndef MS5611_STATIC_ADDRESS
  Ms5611(uint16_t twAddress = ms5611DefaultAdress) : address(twAddress) { } //Address set with constructor
#endif
  void init(void);
  void computeMeasures(uint8_t* d1Buff, uint8_t* d2Buff, double& temperature, double& pressure);
  static double computeAltitude(double pressure);
  void readHardwareCalibration(uint16_t* cal);

 private:
#ifdef MS5611_STATIC_ADDRESS
  static constexpr uint16_t address = MS5611_STATIC_ADDRESS;
#else
  const uint16_t address;
#endif

#ifdef MS5611_STATIC_CALIBRATION
  static constexpr Ms5611Calibration calibration = MS5611_STATIC_CALIBRATION;
#else
  Ms5611Calibration calibration;
#endif
};


#endif //MS5611_H
