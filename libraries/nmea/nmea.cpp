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

#include <nmea.h>

#include <Arduino.h>
#include <digit.h>
#include <avr/pgmspace.h>

#define NMEA_TAG_SIZE 5
#define RMC_SPEED_COMMA_COUNT 7
#define GGA_ALTI_COMMA_COUNT 9

const char rmcTag[] PROGMEM = {"GPRMC"};
const char ggaTag[] PROGMEM = {"GPGGA"};

#define POV_TAG_SIZE 7
#define POV_TAG_FULL_SIZE 9
const char povTag[] PROGMEM = {"$POV,E,\r\n"};

/**************/
/* state byte */
/**************/
#define AVAILABLE 0
#define RMC_FOUND 1
#define GGA_FOUND 2
#define PARITY_FOUND 3
#define VALUE_FOUND 4
#define HAVE_NEW_SPEED_VALUE 5
#define HAVE_NEW_ALTI_VALUE 6
#define READY 7

#define nmeaState_set(bit) state |= (1 << bit)
#define nmeaState_unset(bit) state &= ~(1 << bit)
#define nmeaState_isset(bit) (state & (1 << bit))


Nmea::Nmea() : valueDigit(NMEA_OUTPUT_VALUE_PRECISION) {
  state = 0;
  povTagPos = POV_TAG_FULL_SIZE; //disable tag writing
  speedCalibrationStep = 0;  
}

void Nmea::feed(uint8_t c) {

  /* by default we suppose that this input byte will be outputed */
  outChar = c;
  nmeaState_set(AVAILABLE);
  inParity ^= c;   //parity will be reverted later if must not be done

  switch( c ) {

    /***********************/
    /* specials characters */
    /***********************/
  case '$':
    /* reset variables */
    readPos = 0;
    nmeaState_set(RMC_FOUND);  //will be disabled while reading tag
    nmeaState_set(GGA_FOUND);
    nmeaState_unset(PARITY_FOUND);
    nmeaState_unset(VALUE_FOUND);
    inParity = 0;
    commaCount = 0;
    break;

  case ',':
    commaCount++;

    /* finished tag reading */ 
    if( commaCount == 1 ) {
      if( readPos != NMEA_TAG_SIZE ) {
	nmeaState_unset(RMC_FOUND);
	nmeaState_unset(GGA_FOUND);
      }
    }

    /* starting speed reading */
    else if( commaCount == RMC_SPEED_COMMA_COUNT ) {
      if( nmeaState_isset(RMC_FOUND) ) {
	value = 0.0;
	readPos = 0;
      }
    }

    /* finished speed reading */
    else if( commaCount == RMC_SPEED_COMMA_COUNT + 1 ) {
      if( nmeaState_isset(RMC_FOUND) && readPos != 0 ) { 
	nmeaState_set(VALUE_FOUND);     //sometimes there is no value in the sentence
      }
    }

    /* starting alti reading */
    else if( commaCount == GGA_ALTI_COMMA_COUNT ) {
      if( nmeaState_isset(GGA_FOUND) ) {
	value = 0.0;
	readPos = 0;
	digitParity = 0; //as the value value will be substitued we need it's parity
      }
    }

    /* finished alti reading */
    else if( commaCount == GGA_ALTI_COMMA_COUNT + 1 ) {
      if( nmeaState_isset(GGA_FOUND) && readPos != 0 ) {
	nmeaState_set(VALUE_FOUND);
      }
    }
    break;

  case '*':
    /* start parity reading */
    readPos = 0;
    parityTag = 0;
    nmeaState_set(PARITY_FOUND);
    inParity ^= c;  //revert parity of *
    break;

    /**********************/
    /* parsing characters */
    /**********************/
  default:

    /* no parity case */
    if( ! nmeaState_isset(PARITY_FOUND) ) {

      /* the NMEA tag */
      if( commaCount == 0 ) {
	if( c != pgm_read_byte_near(rmcTag + readPos) ) {
	  nmeaState_unset(RMC_FOUND);
	}
	if( c != pgm_read_byte_near(ggaTag + readPos) ) {
	  nmeaState_unset(GGA_FOUND);
	}
	readPos++;
      }

      /* the speed or alti value */
      if( ! nmeaState_isset(VALUE_FOUND) &&
	  ( commaCount == RMC_SPEED_COMMA_COUNT || commaCount == GGA_ALTI_COMMA_COUNT )) {
	/* made even if it is not the correct tag */
	/* ignore dot as the precision is constant */
	if( c >= '0' && c <= '9' ) {  
	  value *= 10;
	  value += c - '0';
	  readPos++;
	}

	/* for GGA we need to subsitute value */
	if( commaCount == GGA_ALTI_COMMA_COUNT && nmeaState_isset(GGA_FOUND) ) {
	  nmeaState_unset(AVAILABLE); //don't output the digits
	  digitParity ^= c; //store parity to correct the parity tag
	}
      }
    }

    /* parity case */
    else {
      /* !! revert parity !! */
      inParity ^= c;
      
      /****************/
      /* parity check */
      /****************/
      
      /* read parity char */
      parityTag <<= 4;
      if( c <= '9' ) {
	parityTag += c - '0';
      } else {
	parityTag += c - 'A' + 10;
      }
      readPos++;
      
      /* if parity is read, check */
      if( readPos == 2 ) {

	/* ok parity is read */
	nmeaState_unset(PARITY_FOUND);

	/* compare parity */
	if( inParity == parityTag && nmeaState_isset(VALUE_FOUND) ) {

	  /* new speed value */
	  if( nmeaState_isset(RMC_FOUND) ) {
	    nmeaState_set(HAVE_NEW_SPEED_VALUE);
	    gpsSpeed = (value/NMEA_RMC_SPEED_PRECISION)*KNOTS_TO_KMH;
	    /* calibration step */
	    speedCalibrationStep++;
	    if( speedCalibrationStep == NMEA_SPEED_CALIBRATION_PASS ) {
	      nmeaState_set(READY);
	      nmeaState_unset(AVAILABLE); //don't output only the last parity digit
	    }
	  }
	
	  /* new alti value */
	  if( nmeaState_isset(GGA_FOUND) ) {
	    nmeaState_set(HAVE_NEW_ALTI_VALUE);
	    gpsAlti = value/NMEA_GGA_ALTI_PRECISION;
	  }
	}
      }

      /* if GGA don't output parity as it will be substitued */
      if( nmeaState_isset(GGA_FOUND) ) {
	nmeaState_unset(AVAILABLE);
      }
    }
  }

  /* don't output before ready */
  if( ! nmeaState_isset(READY) ) {
    nmeaState_unset(AVAILABLE);
  }
}


boolean Nmea::available(void) {
  
  if(  nmeaState_isset(AVAILABLE) ) {
    return true;
  } else {
    return false;
  }
}


uint8_t Nmea::read(void) {

  /* by default output the outChar */
  /* at the end AVAILABLE will be disabled if no more output characters */
  uint8_t readChar = outChar;

  /*****************************************/
  /* check if we need to launch new output */
  /*****************************************/

  /* need to start gga alti output ? */  
  if(  nmeaState_isset(GGA_FOUND) && commaCount == GGA_ALTI_COMMA_COUNT && outChar == ',' ) {

    /* init speed digit */
    valueDigit.begin(baroAlti);
    outChar = 0;
  }

  /* need to start gga parity output ? */
  else if(  nmeaState_isset(GGA_FOUND) &&  outChar == '*' ) {

    /* compute new parity and init digit */
    parityDigit.begin( inParity ^ digitParity ); //this revert old digit parity and add new digit parity
    outChar = 0;
  }

  /* need to start POV sentence ? */
  else if( nmeaState_isset(GGA_FOUND) && outChar == '\n' ) {

    povTagPos = 0; //will make tag output
    inParity = 0;  //used to compute sentence parity
    digitParity = 0; //parity off vario digit
    valueDigit.begin(baroVario); //displayed just after the tag
    outChar = 1; //used to signal that we output an entire sentence
  }

  /***********************/
  /* outputs by priority */
  /***********************/

  /* special output */
  if( readChar <= 1 ) { 

    /* check the POV tag */
    if( povTagPos < POV_TAG_SIZE ) {
      readChar =  pgm_read_byte_near(povTag + povTagPos);
      if( povTagPos != 0) {  //$ is not in parity
	inParity ^= readChar;
      }
      povTagPos++;
    }

    /* check value digit */
    else if( valueDigit.available() ) {
      readChar = valueDigit.get();
      digitParity ^= readChar;

      /* check if we are writing the POV sentence */
      if( ! valueDigit.available() ) {
	if( outChar == 1 ) {
	  outChar = '*'; //launch parity output
	} else {
	  nmeaState_unset(AVAILABLE);
	}
      }
    }

    /* check parity digit */
    else if( parityDigit.available() ) {
      readChar = parityDigit.get();

      if( ! parityDigit.available() ) {
	/* if we not writing POV sentence */
	if( povTagPos >= POV_TAG_FULL_SIZE  ) {
	  nmeaState_unset(AVAILABLE);
	}
      }
    }

    /* new line */
    else if( povTagPos < POV_TAG_FULL_SIZE ) {
      readChar =  pgm_read_byte_near(povTag + povTagPos);
      povTagPos++;
      if( povTagPos == POV_TAG_FULL_SIZE ) {
	nmeaState_unset(AVAILABLE);
      }
    }
  }
    

  /* standard output */
  else {
    if( outChar == readChar ) {
      nmeaState_unset(AVAILABLE);
    }
  }

  return readChar;
}
    
    
    
boolean Nmea::ready(void) {
  if( nmeaState_isset(READY) ) {
    return true;
  } else {
    return false;
  }
}
  

boolean Nmea::haveNewSpeedValue(void) {
  if( nmeaState_isset(HAVE_NEW_SPEED_VALUE) ) {
    return true;
  } else {
    return false;
  }
}


boolean Nmea::haveNewAltiValue(void) {
  if( nmeaState_isset(HAVE_NEW_ALTI_VALUE) ) {
    return true;
  } else {
    return false;
  }
}


double Nmea::getSpeed(void) {
  nmeaState_unset(HAVE_NEW_SPEED_VALUE);
  return gpsSpeed;
}


double Nmea::getAlti(void) {
  nmeaState_unset(HAVE_NEW_ALTI_VALUE);
  return gpsAlti;
}


void Nmea::setBaroData(double alti, double vario) {
  baroAlti = alti;
  baroVario = vario;
}
