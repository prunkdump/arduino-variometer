#include <digit.h>
#include <Arduino.h>

#define PLUS_DISPLAY 0
#define DISPLAY_PLUS 1
#define DISPLAY_MINUS 2
#define DISPLAYED 3

#define digitState_set(bit) state |= (1 << bit)
#define digitState_unset(bit) state &= ~(1 << bit)
#define digitState_isset(bit) (state & (1 << bit))



/* init */
Digit::Digit(boolean plusDisplay) {

  state = 0;
 
  if( plusDisplay ) {
    digitState_set(PLUS_DISPLAY);
  }

  exp = 0; //exp is used to known if digits are available
}

/**********/
/*  sign  */
/**********/
void Digit::setPositive(void) {
  digitState_unset(DISPLAY_MINUS);
  if( digitState_isset(PLUS_DISPLAY) ) {
    digitState_set(DISPLAY_PLUS);
  }
}

void Digit::setNegative(void) {
  digitState_unset(DISPLAY_PLUS);
  digitState_set(DISPLAY_MINUS);
}

void Digit::applySign(double& value) {
  if( value < 0.0 ) {
    value *= -1.0;
    setNegative();
  } else {
    setPositive();
  }
}

void Digit::applySign(long& value) {
  if( value < 0 ) {
    value *= -1;
    setNegative();
  } else {
    setPositive();
  }
}

/******************/
/* digit exponent */
/******************/
/* compute exponent and dot position for an int */
void Digit::computeExponent(void) {

  /* base pos/exp */
  pos = -1;
  exp = 1;

  /* check number of digits */
  unsigned long val = ival;
  while( val >= 10 ) {
    exp *= 10;
    pos--;
    val /= 10;
  }
}

/*******************************/
/* float precision computation */
/*******************************/

/* apply precision and save precision power */
double Digit::applyPrecision(double value, uint8_t precision) {

  exp = 1; 
  for( uint8_t i = 0; i<precision; i++) {
    value *= 10.0;
    exp *= 10;
  }

  return value;
}

void Digit::buildForPrecision(double value, uint8_t precision) {

  /* check sign */
  applySign(value);
  
  /* save rounded ival */
  ival = (unsigned long)(value + 0.5);

  /* check leading zeros */
   if( ival < exp ) { //exp is supposed precision exponent
    pos = -1;
  } else {
    computeExponent(); //update exp relative to digits
    pos += precision;
  }
}
  

/******************/
/* Public methods */
/******************/

void Digit::begin(double value, uint8_t precision) {

  value = applyPrecision(value, precision);
  buildForPrecision(value, precision);
}


void Digit::begin(unsigned long value) {

  setPositive();
 
  /* save value */
  ival = value;
  
  /* compute exp */
  computeExponent();
}

void Digit::begin(long value) {

  applySign(value);

  /* save value */
  ival = (unsigned long)value;
  
  /* compute exp */
  computeExponent();
}


void FPDigit::begin(double value) {
  Digit::begin(value, precision);
}

bool FPSDigit::begin(double value) {
  
  /* treat value for precision */
  value = applyPrecision(value, precision);

  /* check if the value need to be updated */
  if( value > lastDisplayValue - DIGIT_STABILIZATION_THRESHOLD &&
      value < lastDisplayValue + DIGIT_STABILIZATION_THRESHOLD ) {
    exp = 0;
    return false;
  }
  
  /* save displayed value */
  lastDisplayValue = value;
  digitState_set(DISPLAYED);
  
  /* build */
  buildForPrecision(value, precision);

  /* save displayed value */
  /*
  lastDisplayValue = (double)ival;
  if( digitState_isset(DISPLAY_MINUS) ) {
    lastDisplayValue *= -1.0;
  }
  */
  
  return true;
}

void FPSDigit::rebuild(void) {
  if( digitState_isset(DISPLAYED) ) {
    applyPrecision(0.0, precision); //just to compute precision exponent
    buildForPrecision(lastDisplayValue, precision);
  }
}
  
  
unsigned Digit::size(unsigned signSize, unsigned digitSize, unsigned dotSize) {
  
  unsigned long e = exp;
  unsigned size = 0;
  int8_t currPos = pos;

  /* number of digits */
  while( e > 0 ) {
    e /= 10;
    size += digitSize;
    currPos++;
  }

  /* plus or minus needed ? */
  if( digitState_isset(DISPLAY_PLUS) || digitState_isset(DISPLAY_MINUS) ) {
    size += signSize;
  }

  /* dot needed ? */
  if( currPos > 0 ) {
    size += dotSize;
  }

  return size;
}

boolean Digit::available(void) {
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


unsigned long Digit::getIntegerDigit(void) {

  return ival;
}


void HexDigit::begin(uint8_t hexValue) {
  value = hexValue;
  pos = 2;
}

boolean  HexDigit::available(void) {
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
  
