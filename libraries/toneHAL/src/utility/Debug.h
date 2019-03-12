//#ifndef DEBUG_H
//#define DEGUG_H

/********************/
/*    Version 1.0   */
/*    01/03/2019    */
/********************/

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
#define PROG_DEBUG			  		//debug principal program
#define IMU_DEBUG			  			//debug IMU
#define I2CDEV_SERIAL_DEBUG   //debug I2Cdev
#define DEGUB_SERIAL_NMEA_1		
//#define SCREEN_DEBUG				//debug Screen
#define GPS_DEBUG							//debug GPS
#define BUTTON_DEBUG					//debug Button
#define TONEDAC_DEBUG					//debug Tone
#define MS5611_DEBUG					//debug MS5611
#define KALMAN_DEBUG					//debug Kalman
#define ACCEL_DEBUG						//debug Accel

//#endif
