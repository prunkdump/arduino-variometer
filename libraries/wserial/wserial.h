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

#ifndef WSERIAL_H
#define WSERIAL_H

#include <Arduino.h>

#define SERIAL_COMPUTING_COMPENSATION  4

class WSerial {

 public:
  WSerial(uint8_t txPin);
  void begin(unsigned long speed);
  void write(uint8_t c);

 private:
  uint8_t txBitMask;
  volatile uint8_t* txPortRegister;
  uint16_t txDelay;
  void delay(void);

};

#endif
