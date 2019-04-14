/*********************************************************************************/
/*                                                                               */
/*                           Libraries eepromHal_PRO                             */
/*                                                                               */
/*  version    Date     Description                                              */
/*    1.0    06/02/19                                                            */
/*                                                                               */
/*********************************************************************************/

#include "eepromHAL.h"
#include "eepromHAL_PRO.h"
#include <Arduino.h>

#if defined(ARDUINO_AVR_PRO)
#include <EEPROM.h>

#if not defined(VARIO_SETTINGS_H)
//Monitor Port 
#define SerialPort Serial

#define EEPROM_DEBUG

#endif

uint8_t EepromHal_pro::read(int address)
{
  return EEPROM.read(address);
}

void EepromHal_pro::update(int address, uint8_t value)
{
  EEPROM.update(address, value);
}

void EepromHal_pro::write(int address, uint8_t value)
{
  EEPROM.write(address, value);
}

bool EepromHal_pro::isValid()
{
  return true;
}

void EepromHal_pro::commit()
{
}

EepromHal_pro EEPROMHAL;
#endif //ARDUINO_AVR_PRO
