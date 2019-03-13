#include <FirmwareUpdater.h>

#include <Arduino.h>
#include <toneAC.h>

#include <TwoWireScheduler.h>
#include <LightInvensense.h>

inline bool firmwareUpdateCheckCond(int16_t* iaccel) {

  /* compute z accel */
  double zaccel = ((double)iaccel[2])/LIGHT_INVENSENSE_ACCEL_SCALE;

  /* check orientation (we suppove no mouvements ) */
  if( zaccel <= FIRMWARE_UPDATER_ZACCEL_THRESHOLD ) {
    return true;
  }

  return false;
}


/* vert accel version */
boolean firmwareUpdateCond(void) {

  /* read raw accel */
  int16_t iaccel[3];
  int32_t iquat[4];
 
  int state = -1;
  unsigned long startTime = millis();
  
  while( state != 0  && (millis() - startTime) <= FIRMWARE_UPDATER_MPU_TIMEOUT ) {
    state = fastMPUReadFIFO(NULL,iaccel,iquat);
  }

  if( state != 0 ) {
    return false;
  }
  
  return firmwareUpdateCheckCond(iaccel);
}


/* Two Wire Scheduler version */
boolean firmwareUpdateCondTWS(void) {

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
  
  return firmwareUpdateCheckCond(iaccel);
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
  
  
  
