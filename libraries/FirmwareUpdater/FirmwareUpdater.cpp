/* FirmwareUpdater -- launch bootloader on reversed position startup
 *
 * Copyright 2016-2019 Baptiste PELLEGRIN
 * 
 * This file is part of GNUVario.
 *
 * GNUVario is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNUVario is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <FirmwareUpdater.h>

#include <Arduino.h>
#include <toneAC.h>

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
  
  
  
