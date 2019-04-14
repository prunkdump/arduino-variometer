/*********************************************************************************/
/*                                                                               */
/*                           Libraries eepromHal_PRO                             */
/*                                                                               */
/*  version    Date     Description                                              */
/*    1.0    06/02/19                                                            */
/*                                                                               */
/*********************************************************************************/

#include "eepromHAL.h"
#include "eepromHAL_8266.h"
#include <Arduino.h>

#if defined(ESP8266)
#include <EEPROM.h>
#endif //ESP8266
#ifndef VARIO_SETTINGS_H

//Monitor Port 
#if defined(ESP8266)
#define SerialPort Serial
#elif defined(ESP32)

#elif defined(ARDUINO_ARCH_SAMD)
#define SerialPort SerialUSB
#elif defined(_BOARD_GENERIC_STM32F103C_H_)

#elif defined(ESP8266)
#define SerialPort Serial
#else
#define SerialPort Serial
#endif

#define EEPROM_DEBUG

#endif

uint8_t EepromHal_8266::read(int address)
{
#if defined(ESP8266)
  return EEPROM.read(address);
#endif //ESP8266
}

void EepromHal_8266::update(int address, uint8_t value)
{
#if defined(ESP8266)
  EEPROM.update(address, value);
#endif //ESP8266
}

void EepromHal_8266::write(int address, uint8_t value)
{
#if defined(ESP8266)
  EEPROM.write(address, value);
#endif //ESP8266
}

bool EepromHal_8266::isValid()
{
  return true;
}

void EepromHal_8266::commit()
{
}

#if defined(ESP8266)
EepromHal_8266 EEPROMHAL;
#endif //ESP8266