#ifndef FIRMWARE_UPDATER_H
#define FIRMWARE_UPDATER_H

#include <Arduino.h>

#define FIRMWARE_UPDATER_BOOTLOADER_ADDRESS 0x7800

#define FIRMWARE_UPDATER_ALARM_BEEP_FREQ 700
#define FIRMWARE_UPDATER_ALARM_BEEP_COUNT 3
#define FIRMWARE_UPDATER_ALARM_BEEP_DURATION 400

#define FIRMWARE_UPDATER_MPU_TIMEOUT 1000
#define FIRMWARE_UPDATER_ZACCEL_THRESHOLD -0.95

/* update condition based on accelerometer */
/* !! you need to init vertaccel or TwoWireScheduler first !! */
boolean firmwareUpdateCond(void);
boolean firmwareUpdateCondTWS(void); //Two Wire scheduler version

/* jump to bootloader to update firmware */
void firmwareUpdate(void);

#endif
