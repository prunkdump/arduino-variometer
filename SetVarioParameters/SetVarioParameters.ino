#include <Arduino.h>
#include <VarioSettings.h>
#include <I2Cdev.h>
#include <ms5611.h>
#include <vertaccel.h>
#include <EEPROM.h>
#include <LightInvensense.h>
#include <avr/pgmspace.h>
#include <toneAC.h>
#include <FirmwareUpdater.h>
#include <IGCSentence.h>
#include <digit.h>

/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
/*!!            !!! WARNING  !!!              !!*/
/*!! Before building check :                  !!*/
/*!! libraries/VarioSettings/VarioSettings.h  !!*/
/*!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!*/
Vertaccel vertaccel;


#define BEEP_FREQ 800
const char model[] PROGMEM = VARIOMETER_MODEL;
const char pilot[] PROGMEM = VARIOMETER_PILOT_NAME;
const char glider[] PROGMEM = VARIOMETER_GLIDER_NAME;

IGCHeader header;

void setup() {
  
   /* launch firmware update if needed */
  delay(VARIOMETER_POWER_ON_DELAY);
  Fastwire::setup(FASTWIRE_SPEED, 0);
  vertaccel.init();
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
