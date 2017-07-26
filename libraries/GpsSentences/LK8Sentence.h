#ifndef LK8SENTENCE_H
#define LK8SENTENCE_H

#include <Arduino.h>
#include <digit.h>

/* no special character for alti, just for vario "V" and parity "P" */
#define LK8_SENTENCE_TAG "$LK8EX1,999999,,V,99,999,*P\r\n"
#define LK8_SENTENCE_TAG_SIZE 29
#define LK8_SENTENCE_ALTI_POS 15
#define LK8_SENTENCE_VARIO_POS 16
#define LK8_SENTENCE_PARITY_POS 26
#define LK8_SENTENCE_ALTI_PRECISION 0
#define LK8_SENTENCE_VARIO_PRECISION 0


class LK8Sentence {

 public:
 LK8Sentence() : valueDigit() { } 
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
