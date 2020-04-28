/* VarioSettings -- Main configuration file
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

#ifndef VARIO_SETTINGS_H
#define VARIO_SETTINGS_H

/*----------------------------*/
/*          SOFTWARE          */
/*      Vario parameters      */
/*                            */
/*----------------------------*/

/* Set your personnal info here and launch */
/* the SetVarioParameters Sketch to store  */
/* them in EEPROM.                         */

/* ------ !!! Run SetVarioParameters.ino !!! ---------*/
/*        !!! Only when you change one   !!!          */
/*        !!! of these 3 settings.       !!!          */
#define VARIOMETER_MODEL "GNUVario"
#define VARIOMETER_PILOT_NAME "Prunk Dump"
#define VARIOMETER_GLIDER_NAME "ITV Dolpo 2"
/*        !!! For the others compilation !!!          */
/*------- !!! is sufficient.             !!! ---------*/


/* time zone relative to UTC */
#define VARIOMETER_TIME_ZONE (+2) 

/*********/
/* Beeps */
/*********/

/* The volume of the beeps, max = 10 */
#define VARIOMETER_BEEP_VOLUME 10

/* The variometer react like this according to vertical speed in m/s :        */
/* (near climbing beep is not enabled by default)                             */
/*                                                                            */
/* <--LOW-BEEP--|------SILENT------|--NEAR-CLIMBING-BEEP--|--CLIMBING-BEEP--> */
/*              |                  |                      |                   */
/*           SINKING         CLIMBING-SENSITIVITY      CLIMBING               */
#define VARIOMETER_SINKING_THRESHOLD -2.0
#define VARIOMETER_CLIMBING_THRESHOLD 0.2
#define VARIOMETER_NEAR_CLIMBING_SENSITIVITY 0.5

/* The near climbing alarm : signal that you enter or exit the near climbing zone */
/* The near climbing beep : beep when you are in near climbing zone               */
//#define VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM
//#define VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP

// Reminder beeps on low battery
#define HAVE_LOW_BATTERY_BEEP
#define BATTERY_BEEPS_INTERVAL 30000 // 30000 for prod
#define LOW_BATT_BEEP_THRESOLD 3350 // low voltage thresold in mV

/*******************/
/* Screen behavior */
/*******************/

/* the duration of the two screen pages in milliseconds */
#define VARIOMETER_BASE_PAGE_DURATION 10000
#define VARIOMETER_ALTERNATE_PAGE_DURATION 2000


/********************/
/* Measure behavior */
/********************/

/* Flight start detection conditions :                      */
/* -> Minimum time after poweron in milliseconds            */
/* -> Minimum vertical velocity in m/s (low/high threshold) */
/* -> Minimum ground speed in km/h                          */
#define FLIGHT_START_MIN_TIMESTAMP 15000
#define FLIGHT_START_VARIO_LOW_THRESHOLD (-0.5)
#define FLIGHT_START_VARIO_HIGH_THRESHOLD 0.5
#define FLIGHT_START_MIN_SPEED 10.0

/* Display integrated climb rate or instantaneous values if disabled     */
/* If enabled set the integration time in ms.                            */
/* ! Climb rate integration time must not be more than glide ratio one ! */
//#define VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE
#define VARIOMETER_CLIMB_RATE_INTEGRATION_TIME 2000
#define VARIOMETER_INTEGRATED_CLIMB_RATE_DISPLAY_FREQ 2.0

/* Glide ratio display parameters  */
/* Integration time in ms.         */
#define VARIOMETER_GLIDE_RATIO_INTEGRATION_TIME 15000


/* Set the GPS precision needed to use the GPS altitude value  */
/* to calibrate the barometric altitude.                       */
/*      !!! the best possible precision is 100 !!!             */ 
#define VARIOMETER_GPS_ALTI_CALIBRATION_PRECISION_THRESHOLD 350


/*****************************/
/* SDCard/Bluetooth behavior */
/*****************************/

/* What type of barometric altitude is sent :           */
/* -> Based on international standard atmosphere        */
/* -> Calibrated with GPS altitude                      */
//#define VARIOMETER_SDCARD_SEND_CALIBRATED_ALTITUDE
//#define VARIOMETER_BLUETOOTH_SEND_CALIBRATED_ALTITUDE

/* GPS track recording on SD card starting condition :  */ 
/* -> As soon as possible (GPS fix)                     */
/* -> When flight start is detected                     */
//#define VARIOMETER_RECORD_WHEN_FLIGHT_START

/* What type of vario NMEA sentence is sent by bluetooth. */
/* Possible values are :                                  */
/*  - VARIOMETER_SENT_LXNAV_SENTENCE                      */
/*  - VARIOMETER_SENT_LK8000_SENTENCE                     */
#define VARIOMETER_SENT_LXNAV_SENTENCE

/* When there is no GPS to sync variometer bluetooth sentences */
/* set the delay between sendings in milliseconds.             */ 
#define VARIOMETER_SENTENCE_DELAY 2000


/*----------------------------*/
/*          HARDWARE          */
/*      Vario parameters      */
/*                            */
/*----------------------------*/

/* Comment or uncomment according to  */
/* what you embed in the variometer   */ 
#define HAVE_SPEAKER
#define HAVE_ACCELEROMETER
#define HAVE_SCREEN
#define HAVE_GPS
#define HAVE_SDCARD
#define HAVE_BLUETOOTH
//#define HAVE_VOLTAGE_DIVISOR
#define HAVE_INTERNAL_VOLTMETER // For 5V chips powered straight from battery

/* ms5611 parameters */
/* You can set the calibration coefficients if known */
#define MS5611_STATIC_ADDRESS 0x77
//#define MS5611_STATIC_CALIBRATION {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}

/* We advice using the ms5611 barometer */
/* but if you want to use the BMP280 you can enable it here */
//#define HAVE_BMP280
#define BMP280_STATIC_ADDRESS 0x76
//#define BMP280_STATIC_CALIBRATION {{0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}

/* If you embed an accelerometer set the model here. */
/* Possible values are :                             */
/*   MPU6050, MPU6500, MPU9150, MPU9250              */
#define MPU9250
#define MPU_STATIC_ADDRESS 0x68

/* calibration method */
// comment this following line to use EEPROM instead of static values
#define VERTACCEL_STATIC_CALIBRATION
#define VERTACCEL_GYRO_CAL_BIAS {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define VERTACCEL_ACCEL_CAL_BIAS {0, 0, 0}
#define VERTACCEL_ACCEL_CAL_SCALE 0
#define VERTACCEL_MAG_CAL_BIAS {0, 0, 0}
#define VERTACCEL_MAG_CAL_PROJ_SCALE -16689

/* enable if you want to use the MPU INT pin */
//#define MPU_ENABLE_INT_PIN
#define MPU_INT_PIN 2

/* Set the pins used for Screen and SD card modules */
#define VARIOSCREEN_DC_PIN 6
#define VARIOSCREEN_CS_PIN 7
#define VARIOSCREEN_RST_PIN 8
#define SDCARD_CS_PIN 14
#define VOLTAGE_DIVISOR_PIN 16

/* time needed to power on all the devices */
#define VARIOMETER_POWER_ON_DELAY 2000

/* The screen contrast */
#define VARIOSCREEN_CONTRAST 60

/* The voltage divisor */
#define VOLTAGE_DIVISOR_VALUE 1.27
#define VOLTAGE_DIVISOR_REF_VOLTAGE 3.3

/* The bauds rate used by the GPS and Bluetooth modules. */
/* GPS and bluetooth need to have the same bauds rate.   */
#define GPS_BLUETOOTH_BAUDS 9600

/* The GPS period in ms                             */
/* use the gps-time-analysis sketch to determine it */
#define GPS_PERIOD 1000

/* The GPS RMC and GGA sentences parameters */
/* Check the SD card ouput or bluetooth output of gps-time-analysis */
#define NMEA_TAG_SIZE 5
#define NMEA_RMC_TAG "GPRMC"
#define NMEA_GGA_TAG "GPGGA"

/* precision = 10^(number of digit after dot in gps ouput) */
/* check the gps-time-analysis output to check the precision */
/* RMC speed is at 8th position */
/* GGA time is at 2th position */
/* GGA Alti is at 10th position */
/* be carefull, time precision is an int */
#define NMEA_RMC_SPEED_PRECISION 1000.0
#define NMEA_GGA_TIME_PRECISION 100
#define NMEA_GGA_ALTI_PRECISION 10.0

/*********************/
/* TWO WIRE settings */
/*********************/

/* Set the freq */
#define VARIO_TW_FREQ 400000UL


#endif
