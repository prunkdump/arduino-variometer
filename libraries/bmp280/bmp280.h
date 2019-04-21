/* bmp280 -- bmp280 interrupt safe library 
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

#ifndef BMP280_H
#define BMP280_H

#include <Arduino.h>
#include <VarioSettings.h>


/* the normalized sea level pressure */  
#define BMP280_BASE_SEA_PRESSURE 1013.25


/*############################################*/
/* You can compile bmp280 with static address */
/* or static calibration coefficients.        */
/* For this define the values with :          */
/*                                            */
/* #define BMP280_STATIC_ADDRESS              */
/* or                                         */
/* #define BMP280_STATIC_CALIBRATION          */
/*                                            */
/*############################################*/

static constexpr uint16_t bmp280DefaultAdress = 0x77;

struct Bmp280Calibration {
  
  uint16_t coeffs[12];
};


/*####################################*/
/* Here the bmp280 hardware constants */
/*####################################*/

#define BMP280_DIG_REG 0x88
#define BMP280_CTRL_MEAS_REG 0xF4
#define BMP280_PRESS_REG 0xF7
#define BMP280_TEMP_REG 0xFA

#define BMP280_MEASURE_CONFIG 0x55
#define BMP280_MEASURE_DELAY 44

/*-----------------------*/
/*                       */
/*     The main class    */
/*                       */
/*-----------------------*/
class Bmp280 {

 public:
#ifndef BMP280_STATIC_ADDRESS
  Bmp280(uint16_t twAddress = bmp280DefaultAdress) : address(twAddress) { } //Address set with constructor
#endif
  void init(void);
  void computeMeasures(uint8_t* pressBuff, uint8_t* tempBuff, double& temperature, double& pressure);
  static double computeAltitude(double pressure);
  void readHardwareCalibration(uint16_t* cal);

  //!!!
  //private:
#ifdef BMP280_STATIC_ADDRESS
  static constexpr uint16_t address = BMP280_STATIC_ADDRESS;
#else
  const uint16_t address;
#endif

#ifdef BMP280_STATIC_CALIBRATION
  static constexpr Bmp280Calibration calibration = BMP280_STATIC_CALIBRATION;
#else
  Bmp280Calibration calibration;
#endif
};

#endif
