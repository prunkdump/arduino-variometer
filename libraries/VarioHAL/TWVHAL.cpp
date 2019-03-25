#include <TWVHAL.h>
#include <Arduino.h>

#include <VarioSettings.h>

TWVHAL VarioTW;

/************************/
/* IntTW implementation */
/************************/
#ifdef VARIO_TW_USE_INTTW

#include <IntTW.h>

void TWVHAL::begin() {

  intTW.begin();
}

bool TWVHAL::writeBytes(uint8_t address, uint8_t cmd, uint8_t count, uint8_t* buff) {

  return intTW.writeBytes(address, cmd, count, buff);
}

bool TWVHAL::readBytes(uint8_t address, uint8_t cmd, uint8_t count, uint8_t* buff) {

  return intTW.readBytes(address, cmd, count, buff);
}

#endif


/*******************************/
/* Arduino Wire implementation */
/*******************************/
#ifdef VARIO_TW_USE_WIRE

#include <Wire.h>

void TWVHAL::begin() {

  Wire.begin();
  Wire.setClock(VARIO_TW_FREQ);
}

bool TWVHAL::writeBytes(uint8_t address, uint8_t cmd, uint8_t count, uint8_t* buff) {

  
  Wire.beginTransmission(address);
  Wire.write(cmd);
  Wire.write(buff, count);
  return Wire.endTransmission() == 0;
}

bool TWVHAL::readBytes(uint8_t address, uint8_t cmd, uint8_t count, uint8_t* buff) {

  Wire.beginTransmission(address);
  Wire.write(cmd);
  if( Wire.endTransmission() == 0 ) {
    if( Wire.requestFrom(address, count) == count ) {
      while( count ) {
	*buff = Wire.read();
	buff++;
	count--;
      }
      return true;
    }
  }

  return false;
}

#endif


/*************************/
/* I2CDev implementation */
/*************************/
#ifdef VARIO_TW_USE_I2CDEV

#include <I2Cdev.h>

void TWVHAL::begin() {

  int adjFreq = VARIO_TW_FREQ/1000UL;
  Fastwire::setup(adjFreq, true);
}

bool TWVHAL::writeBytes(uint8_t address, uint8_t cmd, uint8_t count, uint8_t* buff) {

  return I2Cdev::writeBytes(address, cmd, count, buff);
}

bool TWVHAL::readBytes(uint8_t address, uint8_t cmd, uint8_t count, uint8_t* buff) {

  return I2Cdev::readBytes(address, cmd, count, buff) == count;
}

#endif
