/* toneHAL -- tone HAL
 *
 * Copyright 2019 Jean-philippe GOI
 * 
 * This file is part of toneHAL.
 *
 * ToneHAL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ToneHAL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
 
/*********************************************************************************/
/*                                                                               */
/*                           Libraries ToneHal                                   */
/*                                                                               */
/*  version    Date     Description                                              */
/*    1.0    20/01/19                                                            */
/*    1.1    24/01/19   Réecriture des classes                                   */
/*                      répartition en plusieurs fichiers                        */
/*    1.2    26/01/19   Modifications mineures                                   */
/*    1.3    09/02/19   Ajout TONEHAL_EXTENDED_VOLUME							               */
/*    1.4    02/03/19   Ajout ESP32                                              */
/*    1.4.1  12/03/19   Modifications mineures								                   */
/*    1.4.2  16/03/19   Ajout description et licence en début de fichier         */
/*                                                                               */
/*********************************************************************************/

#include "toneHAL.h"
#include <Arduino.h>

#if defined(ESP8266)
#elif defined(ESP32)
//********************
// ARDUINO ESP32
//********************

#include "toneHAL_ESP32.h"

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

#if defined (TONEHAL_EXTENDED_VOLUME)

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

#endif //TONEHAL_EXTENDED_VOLUME

