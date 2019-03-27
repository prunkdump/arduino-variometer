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

#ifndef FIRMWARE_UPDATER_TWS_H
#define FIRMWARE_UPDATER_TWS_H

#include <Arduino.h>
#include <VarioSettings.h>

#define FIRMWARE_UPDATER_BOOTLOADER_ADDRESS 0x7800

#define FIRMWARE_UPDATER_ALARM_BEEP_FREQ 700
#define FIRMWARE_UPDATER_ALARM_BEEP_COUNT 3
#define FIRMWARE_UPDATER_ALARM_BEEP_DURATION 400

#define FIRMWARE_UPDATER_MPU_TIMEOUT 1000
#define FIRMWARE_UPDATER_ZACCEL_THRESHOLD -0.95

#ifdef HAVE_ACCELEROMETER
/* update condition based on accelerometer */
/* !! you need to init TwoWireScheduler first !! */
boolean firmwareUpdateCondTWS(void); //Two Wire scheduler version

/* jump to bootloader to update firmware */
void firmwareUpdate(void);
#endif //HAVE_ACCELEROMETER

#endif
