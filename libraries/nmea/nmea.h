/* nmea -- NMEA sentence parser 
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

#ifndef NMEA_H
#define NMEA_H

#include <Arduino.h>
#include <digit.h>

#define NMEA_OUTPUT_VALUE_PRECISION 1
#define NMEA_SPEED_CALIBRATION_PASS 5

/* number of digit after dot in gps input = 1/precision */
#define NMEA_RMC_SPEED_PRECISION 1000.0
#define NMEA_GGA_ALTI_PRECISION 10.0

#define KNOTS_TO_KMH 1.852 

/************************************************************/
/* NMEA                                                     */ 
/* -> parse the nmea sentences from gps                     */
/* -> get the base alti from gps                            */
/* -> substitute the barometric alti from next sentences    */
/* -> get speed from gps                                    */
/* -> add a openvario sentence for vario value              */
/************************************************************/

class Nmea {

 public:
  Nmea();
  void feed(uint8_t c);                   //feed data from GPS. ONE BYTE AT TIME and check available.
  boolean available(void);                //can return multiple characters
  uint8_t read(void);
  boolean ready(void);                    //when ready, now output nmea, you can calibrate alti from GPS data
  double getAlti(void);                   //read alti from GPS data
  double getSpeed(void);                  //read speed from GPS data
  boolean haveNewSpeedValue(void);
  boolean haveNewAltiValue(void);
  void setBaroData(double alti, double vario); //has often has possible update baro data

 private:
  uint8_t state;       
  uint8_t outChar;     //default output char
  uint8_t inParity;    //parity from GPS data
  uint8_t readPos;     //bytes read from nmea value or tag
  uint8_t commaCount;  //comma count since sentence beginning
  double value;        //value read from GPS data
  uint8_t digitParity; //the parity get from reading value
  uint8_t parityTag;   //the tag given at the end of nmea sentence
  double gpsSpeed;
  double gpsAlti;

  uint8_t povTagPos;                              //to write "$POV," nmea tag
  Digit valueDigit;  //to output modified alti or vario
  HexDigit parityDigit;                           //to output parity
  
  uint8_t speedCalibrationStep; //number of speed value before ready

  double baroAlti;
  double baroVario;

};

#endif
