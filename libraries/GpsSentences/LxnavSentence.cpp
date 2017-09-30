#include <LxnavSentence.h>
#include <Arduino.h>
#include <digit.h>

const char lxnavTag[] PROGMEM = LXNAV_SENTENCE_TAG;


void LxnavSentence::begin(double startAlti, double startVario) {

  vario = startVario; //vario is in cm/s
  valueDigit.begin(startAlti, LXNAV_SENTENCE_ALTI_PRECISION);
  parity = '$';  //remove characters not in parity, patity computed before '*' 
  tagPos = 0;
}

bool LxnavSentence::available(void) {
  
  if( tagPos < LXNAV_SENTENCE_TAG_SIZE ) {
    return true;
  }

  return false;
}

uint8_t LxnavSentence::get(void) {

  uint8_t outc = 0;

  /****************/
  /* check digits */
  /****************/
  if( valueDigit.available() && tagPos >= LXNAV_SENTENCE_ALTI_POS ) {
    outc = valueDigit.get();
  } else if( parityDigit.available() ) {
    outc =  parityDigit.get();
  }

  /******************/
  /* else write tag */
  /******************/
  else {
    outc = pgm_read_byte_near(lxnavTag + tagPos);
    tagPos++;

    /* check special characters */
    if( tagPos == LXNAV_SENTENCE_VARIO_POS ) {
      valueDigit.begin(vario, LXNAV_SENTENCE_VARIO_PRECISION);
      tagPos++;
    } else if( tagPos == LXNAV_SENTENCE_PARITY_POS ) {
      parityDigit.begin(parity);
      tagPos++;
    }
  }

  /**********/
  /* parity */
  /**********/
  parity ^= outc;
  return outc;
}
  
