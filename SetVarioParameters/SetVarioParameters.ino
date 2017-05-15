#include <Arduino.h>
#include <I2Cdev.h>
#include <ms5611.h>
#include <vertaccel.h>
#include <EEPROM.h>
#include <inv_mpu.h>
#include <avr/pgmspace.h>
#include <toneAC.h>
#include <FirmwareUpdater.h>
#include <IGCSentence.h>
#include <digit.h>

/* set the params here */
#define VARIOMETER_MODEL "GNUVario"
#define PILOT_NAME "Prunk Dump"
#define GLIDER_NAME "ITV Dolpo 2"

#define BEEP_FREQ 800


const char model[] PROGMEM = VARIOMETER_MODEL;
const char pilot[] PROGMEM = PILOT_NAME;
const char glider[] PROGMEM = GLIDER_NAME;

IGCHeader header;

void setup() {
  
  /* wait a little */
  delay(1000);
    
  /* launch firmware update if needed */
  Fastwire::setup(400,0);
  vertaccel_init();
  if( firmwareUpdateCond() ) {
   firmwareUpdate();
  }
  
  /* save params to EEPROM */
  boolean state = header.saveParams(model, pilot, glider);

  /* if OK signal */
  if( state ) {
    delay(1000);
    
    for( int i = 0; i<3; i++) {
      toneAC(BEEP_FREQ);
      delay(200);
      toneAC(0);
      delay(200);
    }
  }
}

void loop() {
  
  
}
