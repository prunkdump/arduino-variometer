/*********************************************************************************/
/*                                                                               */
/*                           Libraries eepromHal_ESP32                           */
/*                                                                               */
/*  version    Date     Description                                              */
/*    1.0    01/04/19                                                            */
/*                                                                               */
/*********************************************************************************/

#include "eepromHAL.h"
#include "eepromHAL_ESP32.h"
#include <Arduino.h>

#if defined(ESP32)
#include <EEPROM.h>

#if not defined(VARIO_SETTINGS_H)
//Monitor Port 
#define SerialPort Serial

#define EEPROM_DEBUG

#endif

#define EEPROM_SIZE 64

EepromHal_ESP32::EepromHal_ESP32(void) 
{
  if (!EEPROM.begin(EEPROM_SIZE))
  {
#if defined (EEPROM_DEBUG) 
    SerialPort.println("failed to initialise EEPROM"); delay(1000000);
#endif //EEPROM_DEBUG
  }	
}

uint8_t EepromHal_ESP32::read(int address)
{
  return EEPROM.read(address);
}

void EepromHal_ESP32::update(int address, uint8_t value)
{
	if (address <= EEPROM_SIZE) {
		EEPROM.write(address, value);
		EEPROM.commit();
	}
}

void EepromHal_ESP32::write(int address, uint8_t value)
{
  if (address <= EEPROM_SIZE) EEPROM.write(address, value);
}

bool EepromHal_ESP32::isValid()
{
  return true;
}

void EepromHal_ESP32::commit()
{
	EEPROM.commit();
}

EepromHal_ESP32 EEPROMHAL;
#endif //ESP32