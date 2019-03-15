/* GPSSentences -- Generate some standard GPS sentences 
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
  
