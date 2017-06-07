#ifndef DIGIT_H
#define DIGIT_H

#include <Arduino.h>

#define DIGIT_STABILIZATION_THRESHOLD 0.7

/* !!! can only display numbers with less than 10 digits !!! */

class Digit {

 public:
  Digit(boolean plusDisplay = false);
  void begin(double value, uint8_t precision); //display double
  void begin(unsigned long value);             //display unsigned integer
  void begin(long value);                      //diplay signed integer
  unsigned size(unsigned signSize = 1, unsigned digitSize = 1, unsigned dotSize = 1); //!! work only just after begin
  boolean availiable(void);
  uint8_t get(void);
  unsigned long getIntegerDigit(void); //the integer formed by the digit displayed

  
 protected:
  void computeExponent(void);
  unsigned long treatPrecision(double& value, uint8_t precision);
  void treatSignLeadingZeros(double& value, unsigned long pExp, uint8_t precision);
  uint8_t state;
  unsigned long ival;
  unsigned long exp;
  int8_t pos;

};

/* fixed precision digit */
class FPDigit : public Digit {
 public:
  FPDigit(uint8_t precision, boolean plusDisplay = false)
    : Digit(plusDisplay), precision(precision) { }

  void begin(double value);
  
 protected:
  uint8_t precision;
};

/* fixed precision stabilized digit */
class FPSDigit : public FPDigit {
 public :
  FPSDigit(uint8_t precision, boolean plusDisplay = false)
    : FPDigit(precision, plusDisplay), lastDisplayValue(100000000000.5) { }

  bool begin(double value); //return true if the stabilized value has changed

 private:
  double lastDisplayValue;

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
