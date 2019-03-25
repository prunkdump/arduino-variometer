#ifndef TWVHAL_H
#define TWVHAL_H

#include <Arduino.h>

//#define VARIO_TW_USE_WIRE
#define VARIO_TW_USE_INTTW
//#define VARIO_TW_USE_I2CDEV

class TWVHAL {

 public:
  void begin();
  //return true on success
  bool writeBytes(uint8_t address, uint8_t cmd, uint8_t count, uint8_t* buff);
  bool readBytes(uint8_t address, uint8_t cmd, uint8_t count, uint8_t* buff);

};

extern TWVHAL VarioTW;

#endif
