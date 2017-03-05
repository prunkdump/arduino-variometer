#include <wserial.h>

#include <Arduino.h>
#include <util/delay_basic.h>

WSerial::WSerial(uint8_t txPin) {
  pinMode(txPin, OUTPUT);
  txBitMask = digitalPinToBitMask(txPin);
  txPortRegister = portOutputRegister( digitalPinToPort(txPin) );
}

void WSerial::begin(unsigned long speed) {

  txDelay = (F_CPU/speed/4L); //avr delay function use 4 clock cycle
  txDelay -= SERIAL_COMPUTING_COMPENSATION; //the time to compute the bit to send
}

void WSerial::write(uint8_t c) {

  volatile uint8_t* reg = txPortRegister;
  uint8_t mask = txBitMask;
  uint8_t invMask = ~txBitMask;
  uint8_t oldSREG = SREG;

  /* disable interrupts */
  cli();  

  /* start bit */
  *reg &= invMask;
  this->delay();

  /* the 8 bits */
  for (uint8_t i = 0; i < 8; i++) {
 
    if (c & 0x1) {
      *reg |= mask; // send 1
    } else {
      *reg &= invMask; // send 0
    }
    this->delay();
    c >>= 1;
  }

  /* end bit and enable interrupts */
  *reg |= mask;
  SREG = oldSREG; 
  this->delay();
}

void WSerial::delay(void) {
  
  _delay_loop_2(txDelay);
}


  
