#ifndef LXNAV_SENTENCE_H
#define LXNAV_SENTENCE_H

#include <Arduino.h>
#include <digit.h>

#define LXNAV_SENTENCE_ALTI_PRECISION 1
#define LXNAV_SENTENCE_VARIO_PRECISION 2
#define LXNAV_SENTENCE_ALTI_POS 3
#define LXNAV_SENTENCE_VARIO_POS 4
#define LXNAV_SENTENCE_PARITY_POS 12
#define LXNAV_SENTENCE_TAG {"$LXWP0,N"}
#define LXNAV_SENTENCE_TAG_SIZE 8
#define LXNAV_SENTENCE_TAG_PARITY 'L'^'X'^'W'^'P'^'0'^','^'N'

class LxnavSentence {

 public:
 LxnavSentence() : valueDigit() { } 
  void begin(double alti, double vario);
  bool availiable(void);
  uint8_t get(void);

 private:
  double vario; //alti is directly initialized
  Digit valueDigit;
  uint8_t parity;
  HexDigit parityDigit;
  uint8_t commaCount;
  uint8_t tagPos;
  
};

#endif
