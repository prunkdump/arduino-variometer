#include <Arduino.h>
#include <VarioSettings.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <ms5611_zero.h>
#include <GenClock_zero.h>


#define UPDATE_PERIOD 1000
unsigned long lastUpdate;

void setup() {
  
  Serial.begin(9600);
  while(!Serial) { }
  Serial.println("Start !"); 

  pinMode(VARIO_PIN_ALIM, OUTPUT);
  digitalWrite(VARIO_PIN_ALIM, HIGH);   // turn on power cards )

  delay(1000);

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
