#include <IGCSentence.h>

#include <Arduino.h>

uint8_t IGCSentence::begin(double baroAlti) {

  /* clear variables */
  outc = 0;
  commaCount = 1;  //TAG not feeded
  digitCount = IGC_SENTENCE_TIME_SIZE; //we start immediately with time

  /* no negative alti */
  uint16_t intAlti;
  if( baroAlti >= 0.0 ) {
    intAlti = (uint16_t)baroAlti;
  } else {
    intAlti = 0;
  }

  /* fill in reverse order */
  for( int i = 0; i<IGC_SENTENCE_ALTI_SIZE; i++ ) {
    digitBuffer[i] = '0' + (intAlti % 10);
    intAlti /= 10;
  }

  return 'B';
}

void IGCSentence::feed(uint8_t c) {

  outc = 0;
  
  /* when receive commma, reinit vars */
  if( c == ',' ) {
    commaCount++;
        
    if( commaCount == 2 ) {
      digitCount = IGC_SENTENCE_LAT_SIZE; //latitude
    }

    else if( commaCount == 3 || commaCount == 5 ) {
      digitCount = IGC_SENTENCE_CARDINAL_SIZE; //N,S,E,O
    }

    else if( commaCount == 4 ) {
      digitCount = IGC_SENTENCE_LONG_SIZE; //longitude
    }

    else if( commaCount == 7 ) {
      outc = 'A'; //we have alti
    }

    else if( commaCount == 8 || commaCount == 10 ) {
      outc = digitBuffer[IGC_SENTENCE_ALTI_SIZE - 1]; //first alti digit
      digitCount = IGC_SENTENCE_ALTI_SIZE - 2;  //next digits (used as pointers)
    }

    else if( commaCount == 9 ) {
      digitCount = IGC_SENTENCE_ALTI_SIZE - 1; //we read gps alti in reverse order
    }

    else if( commaCount == 11 ) {
      outc = '\r';
    }

    return;
  }

  /*********************/
  /* there is two case */
  /*********************/
  
  /* -> we are reading GPS alti */
  if( commaCount == 9 ) {
    if( digitCount < IGC_SENTENCE_ALTI_SIZE ) {
      /* if dot was not found get the digit */
      if( c != '.' ) {
	digitBuffer[digitCount] = c;
	digitCount--;
      }

      /* build alti digits */
      else {
	uint8_t i = 0;
	
	/* translate digits */
	digitCount++;
	while( digitCount < IGC_SENTENCE_ALTI_SIZE ) {
	  digitBuffer[i] = digitBuffer[digitCount];
	  i++;
	  digitCount++;
	}

	/* fill with 0 */
	while( i < IGC_SENTENCE_ALTI_SIZE ) {
	  digitBuffer[i] = '0';
	  i++;
	}
      }
    }
  }
  
  /* -> we are outputting digits */
  else {
    if( commaCount < 8 && c != '.' && digitCount ) {
	digitCount--;
	outc = c;
    }
  }
}
 
bool IGCSentence::availiable() {

  return (bool)outc;
}

uint8_t IGCSentence::get() {

  /* get last char */
  uint8_t retChar = outc;
  outc = 0;

  /* check if we need to send next char immediately  */
  if( digitCount >= 0 ) {
    if( commaCount == 8 || commaCount == 10 ) {  //we outputing alti 
      outc = digitBuffer[digitCount];
      digitCount--;
    }
  }

  else {
    digitCount = 0; //don't stay with digitCount = -1 
  }

  /* new line */
  if( retChar == '\r' ) {
    outc = '\n';
  }

  /* send */
  return retChar;
}
    
