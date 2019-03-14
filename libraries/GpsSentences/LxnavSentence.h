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

#ifndef LXNAV_SENTENCE_H
#define LXNAV_SENTENCE_H

#include <Arduino.h>
#include <digit.h>

/* no special character for the first digit (alti), just for vario "V" and parity "P" */
#define LXNAV_SENTENCE_TAG "$LXWP0,Y,,,V,,,,,,,,*P\r\n" 
#define LXNAV_SENTENCE_TAG_SIZE 24
#define LXNAV_SENTENCE_ALTI_POS 10
#define LXNAV_SENTENCE_VARIO_POS 11
#define LXNAV_SENTENCE_PARITY_POS 21
#define LXNAV_SENTENCE_ALTI_PRECISION 1
#define LXNAV_SENTENCE_VARIO_PRECISION 2


class LxnavSentence {

 public:
 LxnavSentence() : valueDigit() { } 
  void begin(double alti, double vario);
  bool available(void);
  uint8_t get(void);

 private:
  double vario; //alti is directly initialized
  Digit valueDigit;
  uint8_t parity;
  HexDigit parityDigit;
  uint8_t tagPos;
};

#endif
