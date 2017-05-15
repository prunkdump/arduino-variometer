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
  bool availiable(void);
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
  bool availiable();
  uint8_t get();

 private:
  uint8_t outc;
  uint8_t commaCount;
  int8_t digitCount;
  uint8_t digitBuffer[IGC_SENTENCE_ALTI_SIZE];
  
};
  


#endif
