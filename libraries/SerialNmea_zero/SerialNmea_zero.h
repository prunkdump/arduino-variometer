#ifndef SERIAL_NMEA_H
#define SERIAL_NMEA_H

#include <Arduino.h>

/**********************/
/*  serial registers  */
/**********************/
#ifdef UCSR0A
#define UCSRA UCSR0A
#define UCSRB UCSR0B
#define UCSRC UCSR0C
#define UDR UDR0
#else
#define UCSRA UCSR1A
#define UCSRB UCSR1B
#define UCSRC UCSR1C
#define UDR UDR1
#endif

#if defined(UBRR0H)
#define UBRRH UBRR0H
#define UBRRL UBRR0L
#elif defined(UBRRH0)
#define UBRRH UBRRH0
#define UBRRL UBRRL0
#elif defined(UBRR1H)
#define UBRRH UBRR1H
#define UBRRL UBRR1L
#else
#define UBRRH UBRRH1
#define UBRRL UBRRL1
#endif

#ifdef RXEN0
#define RXEN RXEN0
#define TXEN TXEN0
#define U2X U2X0
#define RXCIE RXCIE0
#define UDRIE UDRIE0
#define UCSZ1 UCSZ01
#define UCSZ0 UCSZ00
#else
#define RXEN RXEN1
#define TXEN TXEN1
#define U2X U2X1
#define RXCIE RXCIE1
#define UDRIE UDRIE1
#define UCSZ1 UCSZ11
#define UCSZ0 UCSZ10
#endif



/*********************/
/* serial nmea class */
/*********************/

#define SERIAL_NMEA_NEWLINE_LENGTH 2 
#define SERIAL_NMEA_MODE ( _BV(UCSZ1) | _BV(UCSZ0) )
#define SERIAL_NMEA_INT_MODE _BV(TXEN)  

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
