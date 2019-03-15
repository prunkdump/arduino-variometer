/* ms5611 -- ms5611 interrupt safe library 
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

#include <ms5611.h>

#include <Arduino.h>
#include <IntTW.h>

void Ms5611::readHardwareCalibration(uint16_t* cal) {

  uint8_t data[2];
  
  /* read calibration coefficients */
  for(int i = 0; i<6; i++) {

    /* read calibration register */
    intTW.readBytes(address, MS5611_CMD_READ_PROM + (i*2), 2, data);

    /* save coeff */
    cal[i] = ((uint16_t)data[0] << 8) + data[1];
  }
}

void Ms5611::init(void) {

#ifndef MS5611_STATIC_CALIBRATION
  readHardwareCalibration(calibration.coeffs); 
#endif

  /* launch read d1 */
  // This allow to control the ms5611 in only two steps
  // -> read d1 + convert d2
  // -> read d2 + convert d1
  intTW.writeBytes(address, MS5611_CMD_CONV_D1, 0, NULL);
  delay(MS5611_CONV_DELAY);
}

void Ms5611::computeMeasures(uint8_t* d1Buff, uint8_t* d2Buff, double& temperature, double& pressure) {

  /* get raw measures */
  uint32_t d1 = 0;
  uint32_t d2 = 0;
  for(int i = 0; i<3; i++) {
    d1 <<= 8;
    d2 <<= 8;
    d1 += d1Buff[i];
    d2 += d2Buff[i];
  }

  /* compute temperature */
  int32_t dt, temp;
  
  int32_t c5s = calibration.coeffs[4];
  c5s <<= 8;
  dt = d2 - c5s;

  int32_t c6s = calibration.coeffs[5];
  c6s *= dt;
  c6s >>= 23;
  
  temp = 2000 + c6s;

  /* compute compensation */
  int64_t off, sens;
  
  /* offset */
  int64_t c2d = calibration.coeffs[1];
  c2d <<=  16;
  
  int64_t c4d = calibration.coeffs[3];
  c4d *= dt;
  c4d >>= 7;

  off = c2d + c4d;
 
  /* sens */
  int64_t c1d = calibration.coeffs[0];
  c1d <<= 15;

  int64_t c3d = calibration.coeffs[2];
  c3d *= dt;
  c3d >>= 8;
 
  sens = c1d + c3d;

  /* second order compensation */
  int64_t t2, off2, sens2;
 
  if( temp < 2000 ) {
    t2 = dt;
    t2 *= t2;
    t2 >>= 31;
    
    off2 = temp-2000;
    off2 *= off2;
    off2 *= 5;
    sens2 = off2;
    off2 >>= 1;
    sens2 >>= 2;
      
    if( temp < -1500 ){
      int64_t dtemp = temp + 1500;
      dtemp *= dtemp;
      off2 += 7*dtemp;
      dtemp *= 11;
      dtemp >>= 1;
      sens2 += dtemp;
    }
    temp = temp - t2;
    off = off - off2;
    sens = sens - sens2;
  }
  
  /* compute pressure */
  int64_t p;
 
  p = d1 * sens;
  p >>= 21;
  p -= off;
  //p >>= 15 !!! done with doubles, see below

  /* save result */
  temperature = (double)temp/100;
  pressure = ((double)p / (double)((uint16_t)1 << 15))/(double)100;
}


static double Ms5611::computeAltitude(double pressure) {
  
  double alti;
  alti = pow((pressure/(MS5611_BASE_SEA_PRESSURE)), 0.1902949572); //0.1902949572 = 1/5.255
  alti = (1-alti)*(288.15/0.0065);
  return alti;
}
