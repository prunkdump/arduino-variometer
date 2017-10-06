#include <SerialNmea_zero.h>

#include <Arduino.h>
#include <SERCOM.h>
#include <wiring_private.h>

#define DEGUB_SERIAL_NMEA_1


#define NMEA_TAG_SIZE 5
const char rmcTag[] PROGMEM = {"GPRMC"};
const char ggaTag[] PROGMEM = {"GPGGA"};

#define serialState_set(bit) state |= (1 << bit)
#define serialState_unset(bit) state &= ~(1 << bit)
#define serialState_isset(bit) (state & (1 << bit))

#define sbi(reg, bit) reg |= (1 << bit)
#define cbi(reg, bit) reg &= ~(1 << bit)

#define LOCKED 0
#define SENTENCE_LOCKED 1
#define LOCKED_RMC 2
#define LOCKED_GGA 3
#define RMC_TAG_FOUND 4
#define GGA_TAG_FOUND 5
#define RMC_FOUND 6
#define GGA_FOUND 7


//TX du bluetooth à la pin RX du Serial1      pin 13
//RX du bluetooth à la pin TX du SerialNmea   pin 4
//pin 2 PA10_TCC0-W2  TX --- RX BT

//TX du GPS sur la pin RX du SerialNmea       pin 5
//pin 3 PA11_TCC0-W3  RX --- TX GPS
//RX du GPS sur la pin TX du Serial1          pin 14

/* serial class declaration */
SERCOM snsercom(SERCOM4);


SerialNmea serialNmea;
void SERCOM4_Handler() {

  if (snsercom.availableDataUART()) {
    serialNmea.rxCompleteVect();
  }

  if (snsercom.isDataRegisterEmptyUART()) {
    serialNmea.udrEmptyVect();
  }
}

void SerialNmea::begin(unsigned long baud, bool rxEnable) {
   
  /*******************/
  /* hardware config */
  /*******************/
//  pinPeripheral(2, PIO_SERCOM_ALT);
//  pinPeripheral(3, PIO_SERCOM_ALT);
  pinPeripheral(4, PIO_SERCOM_ALT);
  pinPeripheral(5, PIO_SERCOM_ALT);

  snsercom.initUART(UART_INT_CLOCK, SAMPLE_RATE_x16, baud);
  snsercom.initFrame(UART_CHAR_SIZE_8_BITS, LSB_FIRST, SERCOM_NO_PARITY, SERCOM_STOP_BIT_1);
  //snsercom.initPads(PAD_SERIAL1_TX, PAD_SERIAL1_RX);
  snsercom.initPads(UART_TX_PAD_2, SERCOM_RX_PAD_3);
  
  snsercom.enableUART();
  
  /*********************/
  /* init rx/tx buffer */
  /*********************/
  state = 0;
  txTail = 0;
  txHead = 0;
  writePos = 0;
  nmeaParseStep = -1;
}




void SerialNmea::rxCompleteVect(void) {

  /* read */
  uint8_t c = snsercom.readDataUART();
#ifdef DEGUB_SERIAL_NMEA_1
//  Serial.write((char)c);
  stateInterrupt = HIGH;
#endif

  /* check '$' that reset write pos */
  if( c == '$' ) {
    /* reset write pos */
    writePos = txHead;
    nmeaParseStep = 0;
  }   

  /* check if write is needed */
  if( serialState_isset(LOCKED) || nmeaParseStep == -1 ) {
    return;
  }

  /*******************/
  /* write to buffer */
  /*******************/

  /* check if we can write to the buffer, we never use the last byte of the buffer */
  uint8_t nextPos = (writePos + 1) % SERIAL_NMEA_BUFFER_SIZE;
  
  if( nextPos == txTail ||
      ( serialState_isset(SENTENCE_LOCKED) && writePos == readPos ) || //never overwrite reading sentence
      ( serialState_isset(RMC_FOUND) && writePos == rmcPos ) ||        //never overwrite same sentence
      ( serialState_isset(GGA_FOUND) && writePos == ggaPos ) ) {
    /* one character is not saved so stop parser */
    writePos = txHead;
    nmeaParseStep = -1;
    return;
  }
  
  /* write */
  buffer[writePos] = c;
  writePos = nextPos;

  /***************/
  /* nmea parser */
  /***************/

  /* start nmea parser */
  if( c == '$' ) {
    /* prepare nmea parser */
    serialState_set(RMC_TAG_FOUND);
    serialState_set(GGA_TAG_FOUND);
    nmeaParity = '*';  //we will add the '*' on parity
    return;
  }

  /* check if we need to compute parity */
  if( nmeaParseStep <= NMEA_TAG_SIZE + 1 ) { //before or on '*'
    nmeaParity ^= c;
  } else {
    if( c >= '0' ) {  //not for \r or \n
      parityTag <<= 4;
      if( c <= '9' ) {
	parityTag += c - '0';
      } else {
	parityTag += c - 'A' + 10;
      }
    }
    nmeaParseStep++;
  }
  
  /* parse nmea tag */
  if( nmeaParseStep < NMEA_TAG_SIZE ) { // on the nmea tag
    if( c != pgm_read_byte_near(rmcTag + nmeaParseStep) ) {
      serialState_unset(RMC_TAG_FOUND);
    }
    if( c != pgm_read_byte_near(ggaTag + nmeaParseStep) ) {
      serialState_unset(GGA_TAG_FOUND);
    }
    nmeaParseStep++;
    return;
  }

  /* check nmea tag */
  else if( nmeaParseStep == NMEA_TAG_SIZE ) { //on the ',' just after the tag
   
    if( c != ',' || (! serialState_isset(RMC_TAG_FOUND) && ! serialState_isset(GGA_TAG_FOUND) ) ) {
      nmeaParseStep = -1; //bad sensence
      writePos = txHead;
    }

    else {
      /* save pointer */
      nmeaPos = nextPos;
      nmeaParseStep++;
    }
    return;
  }

  /* wait for parity */
  else if( nmeaParseStep == NMEA_TAG_SIZE + 1 ) { //before or on dot
   
    if( c == '*' ) {
      parityTag = 0;
      nmeaParseStep++;
      return;
    }
  }

  /* check parity */
  else if ( nmeaParseStep == NMEA_TAG_SIZE + 4 + SERIAL_NMEA_NEWLINE_LENGTH ) { //parity tag read + newline
     
    if( nmeaParity != parityTag ) {
      nmeaParseStep = -1; //bad sensence
      writePos = txHead;
    } else {
      /*********************************/
      /* we have a new nmea sentence ! */
      /*********************************/

      /* save pointer */
      if( serialState_isset(RMC_TAG_FOUND) ) {
	rmcPos = nmeaPos;
	serialState_set(RMC_FOUND);
      } else { //this is gga
	ggaPos = nmeaPos;
	serialState_set(GGA_FOUND);
      }

      /* send with serial */
      txHead = nextPos;
      snsercom.enableDataRegisterEmptyInterruptUART();
    }
  }
}


void SerialNmea::udrEmptyVect(void) {

  /* send */
  uint8_t c = buffer[txTail];
  txTail = (txTail + 1) % SERIAL_NMEA_BUFFER_SIZE;

  snsercom.writeDataUART(c);

  /* if no remaining bytes, stop interrupt */
  if (txHead == txTail) {
    snsercom.disableDataRegisterEmptyInterruptUART();
  }
}


bool SerialNmea::lockRMC(void) {

  /* check if we have RMC sentence */
  if( ! serialState_isset(RMC_FOUND) ) {
    return false;
  }

  /* get the RMC pointer and lock */
  readPos = rmcPos;
  serialState_set(SENTENCE_LOCKED);
  serialState_set(LOCKED_RMC);

  return true;
}

bool SerialNmea::lockGGA(void) {
  /* check if we have RMC sentence */
  if( ! serialState_isset(GGA_FOUND) ) {
    return false;
  }

  /* get the RMC pointer and lock */
  readPos = ggaPos;
  serialState_set(SENTENCE_LOCKED);
  serialState_set(LOCKED_GGA);
  
  return true;
}


void SerialNmea::release() {
  
  if( serialState_isset(LOCKED_RMC) ) {
    serialState_unset(RMC_FOUND);
    serialState_unset(LOCKED_RMC);
  }

  else if( serialState_isset(LOCKED_GGA) ) {
    serialState_unset(GGA_FOUND);
    serialState_unset(LOCKED_GGA);
  }

  serialState_unset(SENTENCE_LOCKED);
  serialState_unset(LOCKED);
}

uint8_t SerialNmea::read(void) {

  uint8_t c = buffer[readPos];
  readPos = (readPos + 1) % SERIAL_NMEA_BUFFER_SIZE;

  return c;
}

void SerialNmea::lock(void) {

  serialState_set(LOCKED);

  /* clear if partial sentences are parsed */
  nmeaParseStep = -1;
  writePos = txHead;
  
}

void SerialNmea::write(uint8_t c) {

  /* check if we can write to the buffer, we never use the last byte of the buffer */
  uint8_t nextPos = (writePos + 1) % SERIAL_NMEA_BUFFER_SIZE;

  if( nextPos == txTail ) {
    return; //write lost
  }

  /* write */
  buffer[writePos] = c;
  writePos = nextPos;
  txHead = nextPos;
  snsercom.enableDataRegisterEmptyInterruptUART();
}




  
