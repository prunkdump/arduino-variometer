/* NmeaParser -- Parse NMEA GPS sentences
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

#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#include <Arduino.h>
#include <VarioSettings.h>


/*********************/
/* the GPS sentences */
/*********************/

/* precision = 10^(number of digit after dot in gps ouput) */
#ifndef NMEA_RMC_SPEED_PRECISION
#define NMEA_RMC_SPEED_PRECISION 1000.0
#endif

#ifndef NMEA_GGA_TIME_PRECISION
#define NMEA_GGA_TIME_PRECISION 100
#endif

#ifndef NMEA_GGA_ALTI_PRECISION
#define NMEA_GGA_ALTI_PRECISION 10.0
#endif

/* position in NMEA sentence */
#define NMEA_PARSER_RMC_SPEED_POS 7
#define NMEA_PARSER_RMC_DATE_POS 9
#define NMEA_PARSER_GGA_TIME_POS 1
#define NMEA_PARSER_GGA_SATELLITE_COUNT_POS 7
#define NMEA_PARSER_GGA_PRECISION_POS 8
#define NMEA_PARSER_GGA_ALTITUDE_POS 9


class NmeaParser {

 public :
 NmeaParser() : satelliteCount(0), state(0) { }
  uint8_t satelliteCount;
  uint32_t time;
  uint32_t date;
  uint16_t precision;
  uint16_t altitude;
  uint16_t speed;

  void beginRMC(void);
  void beginGGA(void);
  void feed(uint8_t c);
  bool haveNewAltiValue(void);
  bool haveNewSpeedValue(void);
  bool haveDate(void);
  double getAlti(void);
  double getSpeed(void); //in KMH
  bool isParsing(void);
  bool isParsingRMC(void);
  bool isParsingGGA(void);

 private :
  uint8_t state;
  uint8_t commaCount;
  uint32_t value;

};
  
  
  


#endif
