/* Digit -- Get digits of numbers
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
  boolean available(void);
  uint8_t get(void);
  unsigned long getIntegerDigit(void); //the integer formed by the digit displayed

  
 protected:
  void setPositive(void);
  void setNegative(void);
  void applySign(double& value);
  void applySign(long& value);
  void computeExponent(void);
  double applyPrecision(double value, uint8_t precision);
  void buildForPrecision(double value, uint8_t precision);
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
  void rebuild(void); //rebuild with the last displayed value

 private:
  double lastDisplayValue;

};
     
  


class HexDigit {

 public:
  void begin(uint8_t hexValue);
  boolean available(void);
  uint8_t get(void);

 private:
  uint8_t value;
  uint8_t pos;

};


#endif
