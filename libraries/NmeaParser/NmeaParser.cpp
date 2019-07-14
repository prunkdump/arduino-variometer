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

#include <NmeaParser.h>

#include <Arduino.h>

#define parserState_set(bit) state |= (1 << bit)
#define parserState_unset(bit) state &= ~(1 << bit)
#define parserState_isset(bit) (state & (1 << bit))

#define PARSE_RMC 0
#define PARSE_GGA 1
#define DIGIT_PARSED 2
#define HAVE_NEW_SPEED_VALUE 3
#define HAVE_NEW_ALTI_VALUE 4
#define HAVE_DATE 5

#define KNOTS_TO_KMH 1.852 

void NmeaParser::beginRMC(void) {

  commaCount = 0;
  value = 0;
  parserState_set(PARSE_RMC);
  parserState_unset(DIGIT_PARSED);
}

void NmeaParser::beginGGA(void) {

  commaCount = 0;
  value = 0;
  parserState_set(PARSE_GGA);
  parserState_unset(DIGIT_PARSED);
}

void NmeaParser::feed(uint8_t c) {

  /* maybe we need to stop parsing */
  if( c == '*' ) {
    parserState_unset(PARSE_RMC);
    parserState_unset(PARSE_GGA);
  }

  if( ! isParsing() ) {
    return;
  }

  /* digit case */
  if( c != ',' ) {
    if( c >= '0' && c <= '9' ) {  
      value *= 10;
      value += c - '0';
      parserState_set(DIGIT_PARSED);
    }
  }

  /* comma case */
  else {
    commaCount++;
   
    /* if we have a value to parse */
    if( parserState_isset(DIGIT_PARSED) ) {

      /* RMC case */
      if( parserState_isset(PARSE_RMC) ) {

	/* RMC speed */
	if( commaCount == NMEA_PARSER_RMC_SPEED_POS ) { 
	  speed = value;
	  parserState_set(HAVE_NEW_SPEED_VALUE);
	}

	/* RMC date */
        else if( commaCount == NMEA_PARSER_RMC_DATE_POS ) {
	  date = value;
	  parserState_set(HAVE_DATE);
	}

      }

      /* GGA case */
      else {

	/* GGA time */
	if( commaCount == NMEA_PARSER_GGA_TIME_POS ) {
	  time = value/NMEA_GGA_TIME_PRECISION;
	}

	/* GGA satellite count */
	else if( commaCount == NMEA_PARSER_GGA_SATELLITE_COUNT_POS ) {
	  satelliteCount = value;
	}

	/* GGA precision */
	else if( commaCount == NMEA_PARSER_GGA_PRECISION_POS ) {
	  precision = value;
	}

	/* GGA altitude */
	else if( commaCount == NMEA_PARSER_GGA_ALTITUDE_POS ) {
	  altitude = value;
	  parserState_set(HAVE_NEW_ALTI_VALUE);
	}

      }
    }

    /* reset value */
    value = 0;
    parserState_unset(DIGIT_PARSED);
  }
}	

  
bool NmeaParser::haveNewAltiValue(void) {

  return parserState_isset(HAVE_NEW_ALTI_VALUE);
}

bool NmeaParser::haveNewSpeedValue(void) {

  return parserState_isset(HAVE_NEW_SPEED_VALUE);
}

bool NmeaParser::haveDate(void) {

  return parserState_isset(HAVE_DATE);
}

double NmeaParser::getAlti(void) {

  parserState_unset(HAVE_NEW_ALTI_VALUE);
  return ((double)altitude)/NMEA_GGA_ALTI_PRECISION;
}
  
double NmeaParser::getSpeed(void) {

  parserState_unset(HAVE_NEW_SPEED_VALUE);
  return (((double)speed)/NMEA_RMC_SPEED_PRECISION)*KNOTS_TO_KMH;
}

bool NmeaParser::isParsing(void) {

  return ( parserState_isset(PARSE_RMC) || parserState_isset(PARSE_GGA) );
}

bool NmeaParser::isParsingRMC(void) {

  return parserState_isset(PARSE_RMC);
}
bool NmeaParser::isParsingGGA(void) {

  return parserState_isset(PARSE_GGA);
}



