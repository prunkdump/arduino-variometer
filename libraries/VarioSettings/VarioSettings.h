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
#define VARIOMETER_MODEL "GNUVario"
#define VARIOMETER_PILOT_NAME "Magali"
#define VARIOMETER_GLIDER_NAME "MAC-PARA Muse 3"

/* time zone relative to UTC */
#define VARIOMETER_TIME_ZONE (+2) 

/*********/
/* Beeps */
/*********/

/* The volume of the beeps, max = 10 */
#define VARIOMETER_BEEP_VOLUME 3

/* The variometer react like this according to vertical speed in m/s :        */
/* (near climbing beep is not enabled by default)                             */
/*                                                                            */
/* <--LOW-BEEP--|------SILENT------|--NEAR-CLIMBING-BEEP--|--CLIMBING-BEEP--> */
/*              |                  |                      |                   */
/*           SINKING         CLIMBING-SENSITIVITY      CLIMBING               */
#define VARIOMETER_SINKING_THRESHOLD -2.5
#define VARIOMETER_CLIMBING_THRESHOLD 0.2
#define VARIOMETER_NEAR_CLIMBING_SENSITIVITY 0.5

/* The near climbing alarm : signal that you enter or exit the near climbing zone */
/* The near climbing beep : beep when you are in near climbing zone               */
//#define VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM
//#define VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP


/*******************/
/* Screen behavior */
/*******************/

/* the duration of the two screen pages in milliseconds */
#define VARIOMETER_BASE_PAGE_DURATION 3000
#define VARIOMETER_ALTERNATE_PAGE_DURATION 3000


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
#define FLIGHT_START_MIN_SPEED 8.0

/* Display integrated climb rate or instantaneous values */
//#define VARIOMETER_DISPLAY_INTEGRATED_CLIMB_RATE

/* Integration parameters  (in ms)                  */
/* !     also used for computing glide ratio      ! */
/* ! even if you don't use integrated climb rate  ! */
#define VARIOMETER_INTEGRATION_TIME 5000
#define VARIOMETER_INTEGRATION_DISPLAY_FREQ 2.0

// secondary display
//Display Ratio      1
//display Climb rate 2
//display both       3
#define RATIO_CLIMB_RATE 2

/* Set the GPS precision needed to use the GPS altitude value  */
/* to calibrate the barometric altitude.                       */
/*      !!! the best possible precision is 100 !!!             */ 
#define VARIOMETER_GPS_ALTI_CALIBRATION_PRECISION_THRESHOLD 200


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
#define VARIOMETER_RECORD_WHEN_FLIGHT_START

/* What type of vario NMEA sentence is sent by bluetooth. */
/* Possible values are :                                  */
/*  - VARIOMETER_SENT_LXNAV_SENTENCE                      */
/*  - VARIOMETER_SENT_LK8000_SENTENCE                     */
#define VARIOMETER_SENT_LK8000_SENTENCE

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
//#define HAVE_BLUETOOTH
#define HAVE_VOLTAGE_DIVISOR

#define HAVE_SCREEN_JPG63

/* If you embed an accelerometer set the model here. */
/* Possible values are :                             */
/*   MPU6050, MPU6500, MPU9150, MPU9250              */
#define MPU9250

/* calibration method */
// by EEPROM
//#define IMU_CALIBRATION_IN_EEPROM
// or by static value

/* Parametre par defaut */
/*#define IMU_GYRO_CAL_BIAS {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define IMU_ACCEL_CAL_BIAS {0, 0, 0}
#define IMU_ACCEL_CAL_SCALE 0
#define IMU_MAG_CAL_BIAS {0, 0, 0}
#define IMU_MAG_CAL_PROJ_SCALE -166*/

//Version 2
#define IMU_GYRO_CAL_BIAS {0x00, 0x00, 0x1b, 0x92, 0x00, 0x00, 0x23, 0x4f, 0x00, 0x01, 0x1c, 0x7f}
#define IMU_ACCEL_CAL_BIAS {3042, 7981, 1753}
#define IMU_ACCEL_CAL_SCALE -288
#define IMU_MAG_CAL_BIAS {9049, 7449, 6753}
#define IMU_MAG_CAL_PROJ_SCALE -3384

//Version 3
//#define IMU_GYRO_CAL_BIAS {0x00, 0x00, 0x3f, 0xf0, 0xff, 0xff, 0xb8, 0x17, 0xff, 0xff, 0xa8, 0x2c}
//#define IMU_ACCEL_CAL_BIAS {-1943, 4749, -15216}
//#define IMU_ACCEL_CAL_SCALE -140
//#define IMU_MAG_CAL_BIAS {45, 3697, 2482}
//#define IMU_MAG_CAL_PROJ_SCALE -9714

/* Set the pins used for Screen V1 */
//#define VARIOSCREEN_DC_PIN 4
//#define VARIOSCREEN_CS_PIN 3
//#define VARIOSCREEN_RST_PIN 2

/* Set the pins used for Screen V2 */
#define VARIOSCREEN_DC_PIN 2
#define VARIOSCREEN_CS_PIN 3
#define VARIOSCREEN_RST_PIN 4

/* Set the pins used for Screen V3 */
//#define VARIOSCREEN_DC_PIN 6
//#define VARIOSCREEN_CS_PIN 7
//#define VARIOSCREEN_RST_PIN 8

/* Set the pins used for SD card modules */
#define SDCARD_CS_PIN 14
#define VOLTAGE_DIVISOR_PIN 16

/* time needed to power on all the devices */
/* Version 1 et 2                          */
#define VARIOMETER_POWER_ON_DELAY 2000

/* time needed to power on all the devices */
/* Version 3                               */
//#define VARIOMETER_POWER_ON_DELAY 3000

/* The screen contrast */
#define VARIOSCREEN_CONTRAST 60

/* The voltage divisor */
/* Version 1 et 2      */
#define VOLTAGE_DIVISOR_VALUE 1.27
#define VOLTAGE_DIVISOR_REF_VOLTAGE 3.3

/* The voltage divisor */
/* Version 3           */
//#define VOLTAGE_DIVISOR_VALUE 1.27
//#define VOLTAGE_DIVISOR_REF_VOLTAGE 3

/* The bauds rate used by the GPS and Bluetooth modules. */
/* GPS and bluetooth need to have the same bauds rate.   */
#define GPS_BLUETOOTH_BAUDS 9600

/* The GPS period in ms                             */
/* use the gps-time-analysis sketch to determine it */
#define GPS_PERIOD 997.5

/* I2C speed                                   */
/* You can try 800 on <8mhz microcontrollers   */ 
/* (Not always work)                           */
#define FASTWIRE_SPEED 400

/* Alarm */
/* Alarm SDCARD not insert */
#define ALARM_SDCARD
/* Alarm GPS Fix */
#define ALARM_GPSFIX
/* Alarm Fly begin */
#define ALARM_FLYBEGIN

/* Mute */
//#define HAVE_MUTE

/* Speed filtering :                                               */
/* Greater values give smoother speed. The base unit is 2 seconds  */
/* so size = 5 use the last 10 seconds to average speed.           */
#define VARIOMETER_SPEED_FILTER_SIZE 5

#endif
