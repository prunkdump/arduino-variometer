#include <Arduino.h>
#include <VarioSettings.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <ms5611_zero.h>
#include <toneACZero.h>


#define UPDATE_PERIOD 1000
unsigned long lastUpdate;

void setup() {
  
  /* init clock for TCC0 and TCC1 */
  toneAC_initClock();

  /* init wire and ms5611 */
  Wire.begin();
  //Wire.setClock(400000);
  ms5611_init();

  /* init time */
  lastUpdate = millis();
}

void loop() {

  unsigned long time = millis();
  if( time - lastUpdate > UPDATE_PERIOD && ms5611_dataReady() ) {

    /* read value */
    ms5611_updateData();
    double alti = ms5611_getAltitude();

    /* print */
    Serial.println(alti, 2);

    /* next update */
    lastUpdate = millis();
  }
}
