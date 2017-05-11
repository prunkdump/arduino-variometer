#include <LxnavSentence.h>

#include <Arduino.h>
#include <digit.h>

const char lxnavTag[] PROGMEM = LXNAV_SENTENCE_TAG;


void LxnavSentence::begin(double startAlti, double startVario) {

  vario = startVario;
  valueDigit.begin(startAlti, LXNAV_SENTENCE_ALTI_PRECISION);
  parity = '$';  //remove characters not in parity 
  commaCount = 1;
  tagPos = 0;
}

bool LxnavSentence::availiable(void) {
  
  if( commaCount < LXNAV_SENTENCE_PARITY_POS + 4 ) {
    return true;
  }

  return false;
}


uint8_t LxnavSentence::get(void) {

  uint8_t outc = 0;

  /******************/
  /* non comma case */
  /******************/

  /*  first the tag */
  if( tagPos < LXNAV_SENTENCE_TAG_SIZE ) {
    outc = pgm_read_byte_near(lxnavTag + tagPos);
    tagPos++;
  }

  /* next the alti or the vario */
  else if( commaCount >= LXNAV_SENTENCE_ALTI_POS && valueDigit.availiable() ) {
    outc = valueDigit.get();
  }

  /* next the parity character */
  else if( commaCount == LXNAV_SENTENCE_PARITY_POS ) {
    outc = '*';
    parityDigit.begin(parity);
    commaCount++;
  }

  /* finally the parity */
  else if( parityDigit.availiable() ) {
    outc = parityDigit.get();
    if( ! parityDigit.availiable() ) {
      commaCount++; //end sentence
    }
  }

  /* end the new line */
  else if( commaCount == LXNAV_SENTENCE_PARITY_POS + 2 ) {
    outc = '\r';
    commaCount++;
  }

  else if( commaCount == LXNAV_SENTENCE_PARITY_POS + 3 ) {
    outc = '\n';
    commaCount++;
  }
      

  /**************/
  /* comma case */
  /**************/
  if( ! outc ) {
    outc = ',';
    commaCount++;

    /* need to launch vario  ? */
    if( commaCount ==  LXNAV_SENTENCE_VARIO_POS) {
      valueDigit.begin(vario, LXNAV_SENTENCE_VARIO_PRECISION);
    }
  }


  parity ^= outc;
  return outc;
}

    



  
    
