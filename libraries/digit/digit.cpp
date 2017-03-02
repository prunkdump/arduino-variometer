#include <digit.h>
#include <Arduino.h>

#define PLUS_DISPLAY 0
#define DISPLAY_PLUS 1
#define DISPLAY_MINUS 2

#define digitState_set(bit) state |= (1 << bit)
#define digitState_unset(bit) state &= ~(1 << bit)
#define digitState_isset(bit) (state & (1 << bit))


Digit::Digit(uint8_t digitPrecision, boolean plusDisplay) {

  state = 0;
  precision = digitPrecision;

  if( plusDisplay ) {
    digitState_set(PLUS_DISPLAY);
  }

  exp = 0; //exp is used to known if digits are availiable
}

void Digit::begin(double value) {

  /* unset flags */
  digitState_unset(DISPLAY_PLUS);
  digitState_unset(DISPLAY_MINUS);
  
  /* check sign */
  if( value < 0.0 ) {
    value *= -1.0;
    digitState_set(DISPLAY_MINUS);
  } else {
    if( digitState_isset(PLUS_DISPLAY) ) {
      digitState_set(DISPLAY_PLUS);
    }
  }

  /******************/
  /* analyse digits */
  /******************/
  pos = 0;

  /* check before dot digits */
  exp = 10;
  while( exp < value ) {
    exp *= 10;
    pos--;
  }

  exp /= 10;
  pos--;

  /* check after dot digits : precision */
  uint8_t p = precision;
  while( p > 0 ) {
    p--;
    exp *= 10;
    value *= 10.0;
  }

  /* store result as int */
  ival = (unsigned long)value;
}


boolean Digit::availiable(void) {
  if( exp > 0 ) {
    return true;
  } else {
    return false;
  }
}

uint8_t Digit::get(void) {

  /* need to display plus */
  if( digitState_isset(DISPLAY_PLUS) ) {
    digitState_unset(DISPLAY_PLUS);
    return '+';
  }

  /* need to display minus */
  if( digitState_isset(DISPLAY_MINUS) ) {
    digitState_unset(DISPLAY_MINUS);
    return '-';
  }

  /* need to display dot */
  if( pos == 0 ) {
    pos++;
    return '.';
  }

  /* else compute digit */
  uint8_t rchar = '0' + ival/exp;
  ival %= exp;
  exp /= 10;
  pos++;
  return rchar;
}


void HexDigit::begin(uint8_t hexValue) {
  value = hexValue;
  pos = 2;
}

boolean  HexDigit::availiable(void) {
  if( pos > 0 ) {
    return true;
  } else {
    return false;
  }
}

uint8_t HexDigit::get(void) {

  uint8_t rchar = value;

  if( pos == 2 ) {
    rchar >>= 4;
  }

  pos--;
  rchar &= 0x0f;
  
  if( rchar < 0x0a ) {
    return '0' + rchar;
  } else {
    return 'A' + rchar - 0x0a;
  }
}
  
