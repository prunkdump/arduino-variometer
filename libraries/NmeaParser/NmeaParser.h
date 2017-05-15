#ifndef NMEA_PARSER_H
#define NMEA_PARSER_H

#define NMEA_PARSER_RMC_SPEED_POS 7
#define NMEA_PARSER_RMC_DATE_POS 9
#define NMEA_PARSER_GGA_TIME_POS 1
#define NMEA_PARSER_GGA_SATELLITE_COUNT_POS 7
#define NMEA_PARSER_GGA_PRECISION_POS 8
#define NMEA_PARSER_GGA_ALTITUDE_POS 9
#define NMEA_PARSER_RMC_SPEED_PRECISION 1000.0
#define NMEA_PARSER_GGA_ALTITUDE_PRECISION 10.0

#include <Arduino.h>


class NmeaParser {

 public :
 NmeaParser() : state(0), satelliteCount(0) { }
  uint8_t satelliteCount;
  uint16_t time;
  uint16_t precision;
  uint16_t altitude;
  uint16_t speed;

  void beginRMC(void);
  void beginGGA(void);
  void feed(uint8_t c);
  bool haveNewAltiValue(void);
  bool haveNewSpeedValue(void);
  double getAlti(void);
  double getSpeed(void);
  bool isParsing(void);
  bool isParsingRMC(void);
  bool isParsingGGA(void);

 private :
  uint8_t state;
  uint8_t commaCount;
  uint16_t value;

};
  
  
  


#endif
