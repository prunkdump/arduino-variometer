/* SerialNmea -- Read serial while keeping only valid NMEA sentences
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

#ifndef SERIAL_NMEA_H
#define SERIAL_NMEA_H

#include <Arduino.h>
#include <VarioSettings.h>

/*********************/
/* the GPS sentences */
/*********************/

/* the nmea tags */
#ifndef NMEA_TAG_SIZE
#define NMEA_TAG_SIZE 5
#endif

#ifndef NMEA_RMC_TAG
#define NMEA_RMC_TAG "GPRMC"
#endif

#ifndef NMEA_GGA_TAG
#define NMEA_GGA_TAG "GPGGA"
#endif

/* the maximum silent time before the timestamp is updated */
#define SERIAL_NMEA_MAX_SILENT_TIME 5

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

#define SERIAL_NMEA_MODE ( _BV(UCSZ1) | _BV(UCSZ0) )
#define SERIAL_NMEA_INT_MODE _BV(TXEN)  

#define SERIAL_NMEA_BUFFER_SIZE 128

class SerialNmea {

 public:
  void begin(unsigned long baud, bool rxEnable);
  bool lockRMC(void);
  bool lockGGA(void);
  void addTagToRead(void); //by default the tag is not included, call this between lock and read
  uint8_t read(void);
  void lock(void); //lock rx completely to write manually
  void write(uint8_t c);
  void release(void);
  unsigned long getReceiveTimestamp(void);
  unsigned long getLastReceiveTimestamp(void);
  
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
  volatile unsigned long receiveTimestamp;
  volatile unsigned long lastReceiveTimestamp;
};

extern SerialNmea serialNmea;

#endif
