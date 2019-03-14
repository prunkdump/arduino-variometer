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

#ifndef IGC_SENTENCE_H
#define IGC_SENTENCE_H

#include <Arduino.h>


#define IGC_SENTENCE_HEADER_EEPROM_ADDRESS 0x30 
#define IGC_SENTENCE_HEADER_MAX_SIZE (0x200 - 0x30)
#define IGC_SENTENCE_EEPROM_TAG 2568


#define IGC_SENTENCE_ALTI_SIZE 5
#define IGC_SENTENCE_TIME_SIZE 6
#define IGC_SENTENCE_LAT_SIZE 7
#define IGC_SENTENCE_LONG_SIZE 8
#define IGC_SENTENCE_CARDINAL_SIZE 1

class IGCHeader {

 public :
  bool saveParams(const char* model, const char* pilot, const char* glider);
  int16_t begin(void);
  bool available(void);
  uint8_t get(void);
  
 private :
  uint16_t size;
  int addr;

};


/*!!! the first character of begin() is part of the sentence !!!*/
class IGCSentence {

 public:
  uint8_t begin(double baroAlti);
  void feed(uint8_t c);
  bool available();
  uint8_t get();

 private:
  uint8_t outc;
  uint8_t commaCount;
  int8_t digitCount;
  uint8_t digitBuffer[IGC_SENTENCE_ALTI_SIZE];
  
};
  


#endif
