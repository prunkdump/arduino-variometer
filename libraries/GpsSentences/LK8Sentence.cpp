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

#include <LK8Sentence.h>
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
  

  
  
