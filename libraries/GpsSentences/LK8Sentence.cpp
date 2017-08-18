#include <Lk8Sentence.h>
#include <Arduino.h>
#include <digit.h>

const char lk8Tag[] PROGMEM = LK8_SENTENCE_TAG;


void LK8Sentence::begin(double startAlti, double startVario) {

  alti = startAlti;
  vario = startVario*100.0; //vario is in cm/s
  double pressure = LK8_BASE_SEA_PRESSURE * 100.0 * pow(1 - (0.0065/288.15)*startAlti, 5.255); 
  valueDigit.begin(pressure, LK8_SENTENCE_PRESSURE_PRECISION);
  parity = '$';  //remove characters not in parity, parity computed before '*' 
  tagPos = 0;
}

bool LK8Sentence::available(void) {
  
  if( tagPos < LK8_SENTENCE_TAG_SIZE ) {
    return true;
  }

  return false;
}

uint8_t LK8Sentence::get(void) {

  uint8_t outc = 0;

  /****************/
  /* check digits */
  /****************/
  if( valueDigit.available() && tagPos >= LK8_SENTENCE_PRESSURE_POS ) {
    outc = valueDigit.get();
  } else if( parityDigit.available() ) {
    outc =  parityDigit.get();
  }

  /******************/
  /* else write tag */
  /******************/
  else {
    outc = pgm_read_byte_near(lk8Tag + tagPos);
    tagPos++;

    /* check special characters */
    if( tagPos == LK8_SENTENCE_ALTI_POS ) {
      valueDigit.begin(alti, LK8_SENTENCE_ALTI_PRECISION);
      tagPos++;
    } else if( tagPos == LK8_SENTENCE_VARIO_POS ) {
      valueDigit.begin(vario, LK8_SENTENCE_VARIO_PRECISION);
      tagPos++;
    } else if( tagPos == LK8_SENTENCE_PARITY_POS ) {
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
  

  
  
