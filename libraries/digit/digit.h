#ifndef DIGIT_H
#define DIGIT_H

#include <Arduino.h>

class Digit {

 public:
  Digit(uint8_t digitPrecision, boolean plusDisplay = false);
  void begin(double value);
  boolean availiable(void);
  uint8_t get(void);

 private:
  uint8_t state;
  uint8_t precision;
  unsigned long ival;
  unsigned long exp;
  uint8_t pos;

};


class HexDigit {

 public:
  void begin(uint8_t hexValue);
  boolean availiable(void);
  uint8_t get(void);

 private:
  uint8_t value;
  uint8_t pos;

};


#endif
