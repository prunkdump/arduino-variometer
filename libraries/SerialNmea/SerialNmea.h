#ifndef SERIAL_NMEA_H
#define SERIAL_NMEA_H

#include <Arduino.h>

#define SERIAL_NMEA_NEWLINE_LENGTH 2 
#define SERIAL_NMEA_MODE ( _BV(UCSZ01) | _BV(UCSZ00) )
#define SERIAL_NMEA_INT_MODE _BV(TXEN0)  

#define SERIAL_NMEA_BUFFER_SIZE 128

class SerialNmea {

 public:
  void begin(unsigned long baud, bool rxEnable);
  bool lockRMC(void);
  bool lockGGA(void);
  uint8_t read(void);
  void lock(void); //lock rx completely to write manually
  void write(uint8_t c);
  void release(void);
  
  void rxCompleteVect(void);
  void udrEmptyVect(void);
  uint8_t buffer[SERIAL_NMEA_BUFFER_SIZE];

 private :
  volatile uint8_t state;
  volatile uint8_t txTail;
  volatile uint8_t txHead;
  volatile uint8_t writePos;
  volatile uint8_t rmcPos;
  volatile uint8_t ggaPos;
  volatile uint8_t nmeaPos;
  uint8_t readPos;
  volatile int8_t nmeaParseStep;
  volatile uint8_t nmeaParity;
  volatile uint8_t parityTag;
  
};

extern SerialNmea serialNmea;

#endif
