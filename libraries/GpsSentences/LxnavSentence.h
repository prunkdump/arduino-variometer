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
