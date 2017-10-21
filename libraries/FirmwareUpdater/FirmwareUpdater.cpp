#include <FirmwareUpdater.h>

#include <Arduino.h>
#include <toneAC_zero.h>

#include <LightInvensense.h>

boolean firmwareUpdateCond(void) {

  /* read raw accel */
  short iaccel[3];
  long iquat[4];
 
  int state = -1;
  unsigned long startTime = millis();
  
  while( state != 0  && (millis() - startTime) <= FIRMWARE_UPDATER_MPU_TIMEOUT ) {
    state = fastMPUReadFIFO(NULL,iaccel,iquat);
  }

  if( state != 0 ) {
    return false;
  }

  /* compute z accel */
  double zaccel = ((double)iaccel[2])/LIGHT_INVENSENSE_ACCEL_SCALE;

  /* check orientation (we suppove no mouvements ) */
  if( zaccel <= FIRMWARE_UPDATER_ZACCEL_THRESHOLD ) {
    return true;
  }

  return false;
}

void firmwareUpdate(void) {

  /* make alarm */
  for( int i = 0; i<FIRMWARE_UPDATER_ALARM_BEEP_COUNT; i++) {
    toneAC(FIRMWARE_UPDATER_ALARM_BEEP_FREQ);
    delay(FIRMWARE_UPDATER_ALARM_BEEP_DURATION);

    toneAC(0);
    delay(FIRMWARE_UPDATER_ALARM_BEEP_DURATION);
  }

  /* jump to bootloader */
  //TO DO !!!

}
  
  
  
