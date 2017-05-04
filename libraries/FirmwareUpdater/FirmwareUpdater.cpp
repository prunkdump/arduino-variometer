#include <FirmwareUpdater.h>

#include <Arduino.h>
#include <toneAC.h>

#include <inv_mpu.h>
#include <inv_mpu_dmp_motion_driver.h>
#include <vertaccel.h>

boolean firmwareUpdateCond(void) {

  /* read raw accel */
  short iaccel[3];
  long iquat[4];
  unsigned long timestamp;
  short sensors;
  unsigned char fifoCount;

  int state = -1;
  unsigned long startTime = millis();
  
  while( state != 0  && (millis() - startTime) <= FIRMWARE_UPDATER_MPU_TIMEOUT ) {
    state = dmp_read_fifo(NULL,iaccel,iquat,&timestamp,&sensors,&fifoCount);
  }

  if( state != 0 ) {
    return false;
  }

  /* conpute z accel */
  double zaccel = ((double)iaccel[2])/VERTACCEL_ACCEL_SCALE;

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
  
  
  
