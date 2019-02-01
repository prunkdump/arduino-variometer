/*********************************************************************************/
/*                                                                               */
/*                           Libraries ToneHal                                   */
/*                                                                               */
/*  version    Date     Description                                              */
/*    1.0    20/01/19                                                            */
/*    1.1    24/01/19   Réecriture des classes                                   */
/*                      répartition en plusieurs fichiers                        */
/*    1.2    26/01/19   Modifications mineures                                   */
/*                                                                               */
/*********************************************************************************/

#include "toneHAL.h"
#include <Arduino.h>

#if defined(ESP8266)
#elif defined(ESP32)
#elif defined(ARDUINO_AVR_PRO)
//********************
// ARDUINO PRO MINI
//********************

#include "toneHAL_PRO.h"

#elif defined(ARDUINO_ARCH_SAMD)
//********************
// MKR ZERO
//********************

#include "toneHAL_M0.h"
#endif

/***********************************/
void ToneHal::setVolume(uint8_t newVolume) {
/***********************************/

  _volume = newVolume;
}

/***********************************/
uint8_t ToneHal::getVolume() {
/***********************************/

  return _volume;
}

/***********************************/
void ToneHal::mute(bool newMuteState) {
/***********************************/
  /* stop tone if needed */
  if( newMuteState ) {
    noTone();
  }

  /* save */
  _toneMuted = newMuteState;
}
