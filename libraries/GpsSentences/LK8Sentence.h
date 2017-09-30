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
