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
