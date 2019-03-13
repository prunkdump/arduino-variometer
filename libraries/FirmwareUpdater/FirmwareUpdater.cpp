#include <FirmwareUpdater.h>

#include <Arduino.h>
#include <toneAC.h>

#include <TwoWireScheduler.h>

boolean firmwareUpdateCond(void) {

  /* wait for accel */
  unsigned long startTime = millis();
  
  while( (! twScheduler.haveAccel()) && ( (millis() - startTime) <= FIRMWARE_UPDATER_MPU_TIMEOUT )) { }
  if( ! twScheduler.haveAccel() ) {
    return false; //timeout
  }

  /* read raw accel */
  int16_t iaccel[3];
  int32_t iquat[4];
  twScheduler.getRawAccel(iaccel, iquat);
  
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
  cli();
  SP = RAMEND;
  void* bootloader = (void*)FIRMWARE_UPDATER_BOOTLOADER_ADDRESS;
  goto *bootloader;

}
  
  
  
