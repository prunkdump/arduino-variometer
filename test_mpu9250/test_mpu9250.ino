#include <Arduino.h>
#include <VarioSettings.h>
#include <Wire.h>
//#include <I2Cdev.h>
//#include <ms5611_zero.h>
#include <vertaccel.h>
#include <toneACZero.h>
//#include <SparkFunMPU9250-DMP.h>

#define UPDATE_PERIOD 100
unsigned long lastUpdate;
bool readReady;

void setup() {
  Serial.begin(9600);
  while (!Serial) { ;}
  
  /* init clock for TCC0 and TCC1 */
//  toneAC_initClock();

  /* init wire */
//  delay(200);
//  Wire.begin();
//  Wire.setClock(400000);

   Serial.println("before init imu");

  /* init mpu9250 */
   vertaccel_initimu();

   Serial.println("after init imu");

  Serial.println("before init dmp");

  /* init mpu9250 */
   delay(8000);
   vertaccel_initdmp();

   Serial.println("after init dmp");

  /* init time */
  readReady = false;
  lastUpdate = millis();
   Serial.println("before loop");

}

void loop() {

  delay(100);
 // Serial.println("loop");

  /* read FIFO as often as possible */
  if( vertaccel_dataReady() ) {
 /*   readReady = true;
     Serial.println("ready");*/

    /* read value */
    vertaccel_updateData();
    double accel = vertaccel_getValue();

    /* print */
    Serial.println(accel, 2);
     

  }

  /* display *
 /* unsigned long time = millis();
  if( time - lastUpdate > UPDATE_PERIOD && readReady ) {

    /* read value *
    vertaccel_updateData();
    double accel = vertaccel_getValue();

    /* print *
    Serial.println(accel, 2);

    /* next update *
    lastUpdate = millis();
    readReady = false;
  }*/

}
