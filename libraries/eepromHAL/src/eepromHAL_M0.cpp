/*********************************************************************************/
/*                                                                               */
/*                           Libraries eepromHal_M0                              */
/*                                                                               */
/*  version    Date     Description                                              */
/*    1.0    05/02/19                                                            */
/*                                                                               */
/*********************************************************************************/

#include "eepromHAL.h"
#include "eepromHAL_M0.h"
#include <Arduino.h>
#if defined(ARDUINO_ARCH_SAMD)
#include "FlashAsEEPROM.h"
#endif //ARDUINO_ARCH_SAMD

#ifndef VARIO_SETTINGS_H

//Monitor Port 
#if defined(ESP8266)

#elif defined(ESP32)

#elif defined(ARDUINO_ARCH_SAMD)
#define SerialPort SerialUSB
#elif defined(_BOARD_GENERIC_STM32F103C_H_)

#elif defined(ARDUINO_AVR_PRO)
#define SerialPort Serial
#else
#define SerialPort Serial
#endif

#define EEPROM_DEBUG

#endif

uint8_t EepromHal_zero::read(int address)
{
#if defined(ARDUINO_ARCH_SAMD)
  return EEPROM.read(address);
#endif //ARDUINO_ARCH_SAMD
}

void EepromHal_zero::update(int address, uint8_t value)
{
#if defined(ARDUINO_ARCH_SAMD)
  EEPROM.update(address, value);
#endif //ARDUINO_ARCH_SAMD
}

void EepromHal_zero::write(int address, uint8_t value)
{
#if defined(ARDUINO_ARCH_SAMD)
  EEPROM.write(address, value);
#endif //ARDUINO_ARCH_SAMD
}

bool EepromHal_zero::isValid()
{
#if defined(ARDUINO_ARCH_SAMD)
  return EEPROM.isValid();
#endif //ARDUINO_ARCH_SAMD
}

void EepromHal_zero::commit()
{
#if defined(ARDUINO_ARCH_SAMD)
  EEPROM.commit();
#endif //ARDUINO_ARCH_SAMD
}

#if defined(ARDUINO_ARCH_SAMD)
EepromHal_zero EEPROMHAL;
#endif //ARDUINO_ARCH_SAMD
