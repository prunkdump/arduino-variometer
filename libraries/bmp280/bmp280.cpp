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

#include <bmp280.h>

#include <Arduino.h>
#include <IntTW.h>


void Bmp280::readHardwareCalibration(uint16_t* cal) {

  uint8_t data[2];
  
  /* read calibration coefficients */
  for(int i = 0; i<12; i++) {

    /* read calibration register */
    //!!! intTW.readBytes(address, BMP280_DIG_REG + (i*2), 2, data);

    /* save coeff */
    //!!! cal[i] = ((uint16_t)data[1] << 8) + data[0];
    if( intTW.readBytes(address, BMP280_DIG_REG + (i*2), 2, data) ) {
      cal[i] = ((uint16_t)data[1] << 8) + data[0];
    } else {
      cal[i] = 0;
    }
  }
}


void Bmp280::init(void) {

#ifndef BMP280_STATIC_CALIBRATION
  readHardwareCalibration(calibration.coeffs); 
#endif

  /* launch first measure */
  // This allow to control the bmp280 in only one step
  // -> read measure + launch next measure
  uint8_t data = BMP280_MEASURE_CONFIG;
  intTW.writeBytes(address, BMP280_CTRL_MEAS_REG, 1, &data);
  delay(BMP280_MEASURE_DELAY);
}


#define dig_T1 calibration.coeffs[0]
#define dig_T2 ((int16_t)calibration.coeffs[1])
#define dig_T3 ((int16_t)calibration.coeffs[2])
#define dig_P1 calibration.coeffs[3]
#define dig_P2 ((int16_t)calibration.coeffs[4])
#define dig_P3 ((int16_t)calibration.coeffs[5])
#define dig_P4 ((int16_t)calibration.coeffs[6])
#define dig_P5 ((int16_t)calibration.coeffs[7])
#define dig_P6 ((int16_t)calibration.coeffs[8])
#define dig_P7 ((int16_t)calibration.coeffs[9])
#define dig_P8 ((int16_t)calibration.coeffs[10])
#define dig_P9 ((int16_t)calibration.coeffs[11])


void Bmp280::computeMeasures(uint8_t* pressBuff, uint8_t* tempBuff, double& temperature, double& pressure) {
  
  /* read measures */
  int32_t adc_T = 0;
  int32_t adc_P = 0;
  for(int i = 0; i<3; i++) {
    adc_T <<= 8;
    adc_P <<= 8;
    adc_T += tempBuff[i];
    adc_P += pressBuff[i];
  }
  adc_T >>= 4;
  adc_P >>= 4;
  
    
  /* compute temp */
  int32_t t_fine;
  int32_t var1, var2, T;
  var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  temperature  = ( (double)(t_fine * 5 + 128) / (double)(1 << 8) ) / 100.0;

  /* compute press */
  int64_t varL1, varL2, p;
  varL1 = ((int64_t)t_fine) - 128000;
  varL2 = varL1 * varL1 * (int64_t)dig_P6;
  varL2 = varL2 + ((varL1*(int64_t)dig_P5)<<17);
  varL2 = varL2 + (((int64_t)dig_P4)<<35);
  varL1 = ((varL1 * varL1 * (int64_t)dig_P3)>>8) + ((varL1 * (int64_t)dig_P2)<<12);
  varL1 = (((((int64_t)1)<<47)+varL1))*((int64_t)dig_P1)>>33;
  if( varL1 == 0 ) {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p<<31) - varL2)*3125)/varL1;
  varL1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
  varL2 = (((int64_t)dig_P8) * p) >> 19;
  pressure = ((((double)(p + varL1 + varL2) / (double)(1 << 8)) + (double)(((int64_t)dig_P7)<<4)) / (double)(1 << 8)) / 100.0;
}

static double Bmp280::computeAltitude(double pressure) {
  
  double alti;
  alti = pow((pressure/(BMP280_BASE_SEA_PRESSURE)), 0.1902949572); //0.1902949572 = 1/5.255
  alti = (1-alti)*(288.15/0.0065);
  return alti;
}
