#include <lightnmea.h>
#include <avr/pgmspace.h>

const char rmcTag[] PROGMEM = {"GPRMC"};
const char ggaTag[] PROGMEM = {"GPGGA"};
#define RMC_COMMA_COUNT 7
#define GGA_COMMA_COUNT 9
#define NMEA_TAG_SIZE 5

#define KNOTS_TO_KMH 1.852

NmeaParser::NmeaParser() {
  // variables are initialized when '$' char is received
  parserState = 0;
}

boolean NmeaParser::getChar(uint8_t c) {

  switch( c ) {
    /****************************/
    /* check special characters */
    /****************************/
  case '$':
    /* réinit private vars */
    readPos = 0;
    parserState |= (1 << RMC_FOUND_BIT);  //switch to false during tag check
    parserState |= (1 << GGA_FOUND_BIT); 
    parserState &= ~(1 << PARITY_FOUND_BIT);
    parityState = 0;
    commaCount = 0;
    value = 0;
    break;
  case ',':
    /* increase comma count */
    commaCount++;

    /* compute parity */
    parityState = parityState ^ c;

    /* check if start reading RMC */
    if( (parserState & (1<<RMC_FOUND_BIT))  && commaCount == RMC_COMMA_COUNT ) {
      if(readPos == NMEA_TAG_SIZE) { // see how nmea tag are checked
	readPos = 0;
	value = 0;
      } else {
	// wrong NMEA tag
	parserState &= ~(1 << RMC_FOUND_BIT);
      }
    }

    /* check if finish reading RMC with no char */
    else if( (parserState & (1<<RMC_FOUND_BIT)) && commaCount == RMC_COMMA_COUNT + 1 &&
	readPos == 0) {
      parserState &= ~(1 << RMC_FOUND_BIT);
    }
     
    /* check if start reading GGA */
    if( (parserState & (1<<GGA_FOUND_BIT)) && commaCount == GGA_COMMA_COUNT ) {
      if(readPos == NMEA_TAG_SIZE) { // see how nmea tag are checked
	readPos = 0;
	value = 0;
      } else {
	// wrong NMEA tag
	parserState &= ~(1 << GGA_FOUND_BIT);
      }
    }
    
    /* check if finish reading GGA with no char */
    else if( (parserState & (1<<GGA_FOUND_BIT)) && commaCount == GGA_COMMA_COUNT + 1 &&
	readPos == 0) {
      parserState &= ~(1 << GGA_FOUND_BIT);
    }
    
    break;
  case '*':
    /* start reading parity */
    readPos = 0;
    parserState |= (1 << PARITY_FOUND_BIT);
    parityTag = 0;
    break;
  default:
    /**********/
    /* parse */
    /*********/

    if( ! (parserState & (1<<PARITY_FOUND_BIT)) ) {
      
      /* the NMEA tag */
      if( commaCount == 0 ) {
	if( readPos >= NMEA_TAG_SIZE ) {
	  parserState &= ~(1 << RMC_FOUND_BIT);
	  parserState &= ~(1 << GGA_FOUND_BIT);
	} else {
	  if( c != pgm_read_byte_near(rmcTag + readPos) ) {
	    parserState &= ~(1 << RMC_FOUND_BIT);
	  }
	  if( c != pgm_read_byte_near(ggaTag + readPos) ) {
	    parserState &= ~(1 << GGA_FOUND_BIT);
	  }
	}
	readPos++;
      }

      /* the RMC speed value or GGA alti */
      else if( ((parserState & (1<<RMC_FOUND_BIT)) && commaCount == RMC_COMMA_COUNT) ||
	       ((parserState & (1<<GGA_FOUND_BIT)) && commaCount == GGA_COMMA_COUNT) ) {
	//if( c == '.' ) {
	//  readPos++;
	//} else 	if( c >= '0' && c <= '9' ) {
	if( c >= '0' && c <= '9' ) {
	  value *= 10;
	  value += c - '0';
	  readPos++;
	  //if( readPos != 0 ) {
	  //  readPos++; //after comma count
	  //}
	}
      }

      /* compute parity */
      parityState = parityState ^ c;

    } else {
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

	if( parityState == parityTag ) {

	  /******************************************/
	  /* it's ok ! we can send value if we have */
	  /******************************************/
	  if( parserState & (1<<RMC_FOUND_BIT) ) {
	    parserState |= (1 << NEW_SPEED_VALUE_BIT);
	    speedValue = value;
	    return true;
	  } else if( parserState & (1<<GGA_FOUND_BIT) ) {
	    parserState |= (1 << NEW_ALTI_VALUE_BIT);
	    altiValue = value;
	    return true;
	  }
	}
      }
    }
  }

  return false;
}

boolean NmeaParser::haveNewSpeedValue() {
  return (parserState & (1<<NEW_SPEED_VALUE_BIT));
}

boolean NmeaParser::haveNewAltiValue() {
  return (parserState & (1<<NEW_ALTI_VALUE_BIT));
}


double NmeaParser::getSpeed() {
  parserState &= ~(1 << NEW_SPEED_VALUE_BIT);
  return (((double)speedValue)/NMEA_SPEED_PRECISION)*KNOTS_TO_KMH;
}

double  NmeaParser::getAlti() {
  parserState &= ~(1 << NEW_ALTI_VALUE_BIT);
  return ((double)altiValue)/NMEA_ALTI_PRECISION;
}
