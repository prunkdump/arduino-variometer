#ifndef DEBUG_H
#define DEGUG_H

//Monitor Port 
#if defined(ESP8266)

#elif defined(ESP32)
#define SerialPort Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialPort SerialUSB
#elif defined(_BOARD_GENERIC_STM32F103C_H_)

#elif defined(ARDUINO_AVR_PRO)
#define SerialPort Serial
#else
#define SerialPort Serial
#endif

//              DEBUGING MODE
#define IMU_DEBUG			  //debug IMU
#define PROG_DEBUG			  //debug principal program
#define I2CDEV_SERIAL_DEBUG   //debug I2Cdev
#define DEGUB_SERIAL_NMEA_1
//#define SCREEN_DEBUG
#define GPS_DEBUG
#define BUTTON_DEBUG
#define TONEDAC_DEBUG
#define MS5611_DEBUG
#define KALMAN_DEBUG
#define ACCEL_DEBUG

#endif
