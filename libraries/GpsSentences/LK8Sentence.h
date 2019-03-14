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

#ifndef LK8SENTENCE_H
#define LK8SENTENCE_H

#include <Arduino.h>
#include <digit.h>

/* no special character for pressure, just for alti "A", vario "V" and parity "P" */
#define LK8_SENTENCE_TAG "$LK8EX1,,A,V,99,999,*P\r\n"
#define LK8_SENTENCE_TAG_SIZE 24
#define LK8_SENTENCE_PRESSURE_POS 8
#define LK8_SENTENCE_ALTI_POS 9
#define LK8_SENTENCE_VARIO_POS 11
#define LK8_SENTENCE_PARITY_POS 21
#define LK8_SENTENCE_PRESSURE_PRECISION 0
#define LK8_SENTENCE_ALTI_PRECISION 0
#define LK8_SENTENCE_VARIO_PRECISION 0

#define LK8_BASE_SEA_PRESSURE 1013.25

class LK8Sentence {

 public:
 LK8Sentence() : valueDigit() { } 
  void begin(double alti, double vario);
  bool available(void);
  uint8_t get(void);

 private:
  double alti;
  double vario;
  Digit valueDigit;
  uint8_t parity;
  HexDigit parityDigit;
  uint8_t tagPos;
};

#endif
