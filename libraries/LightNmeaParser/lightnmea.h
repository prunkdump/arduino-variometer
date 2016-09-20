#ifndef LIGHT_NMEA_H
#define LIGHT_NMEA_H

#include <Arduino.h>

#define NMEA_SPEED_PRECISION 1000.0
#define NMEA_ALTI_PRECISION 10.0

#define RMC_FOUND_BIT 0
#define GGA_FOUND_BIT 1
#define PARITY_FOUND_BIT 2
#define NEW_SPEED_VALUE_BIT 3
#define NEW_ALTI_VALUE_BIT 4

class NmeaParser {

 public:
  NmeaParser();
  boolean getChar(uint8_t c);
  boolean haveNewSpeedValue();
  boolean haveNewAltiValue();
  double getSpeed();
  double getAlti();

 private:
  /*
  uint8_t readPos;
  uint8_t parserSta
  boolean rmcFound;
  boolean ggaFound;
  boolean parityFound;
  uint8_t parityState;
  unsigned commaCount;
  double value;
  double speedValue;
  double altiValue;
  uint8_t parityTag;
  boolean newSpeedValue;
  boolean newAltiValue;
  */
  
  uint8_t readPos;
  uint8_t parserState;
  uint8_t parityState;
  uint8_t commaCount;
  unsigned value;
  unsigned speedValue;
  unsigned altiValue;
  uint8_t parityTag;
};

#endif
