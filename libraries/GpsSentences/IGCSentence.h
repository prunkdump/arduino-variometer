#ifndef IGC_SENTENCE_H
#define IGC_SENTENCE_H

#include <Arduino.h>


#define IGC_SENTENCE_ALTI_SIZE 5
#define IGC_SENTENCE_TIME_SIZE 6
#define IGC_SENTENCE_LAT_SIZE 7
#define IGC_SENTENCE_LONG_SIZE 8
#define IGC_SENTENCE_CARDINAL_SIZE 1

class IGCHeader {

 public :
  int16_t begin(void);
  bool available(void);
  uint8_t get(void);
  
 private :
  unsigned stringIdx;
  unsigned stringPos;

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
