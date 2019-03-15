/* wserial -- Light write only software serial library  
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


  
