/* toneHAL -- derived class for ProMini
 *
 * Copyright 2019 Jean-philippe GOI
 * 
 * This file is part of ToneHAL.
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
 
#include "toneHAL.h"
#include "toneHAL_PRO.h"

#if defined(ARDUINO_AVR_PRO)
#include <Arduino.h>
//#include "Debug.h"
#include "utility\Debug.h"

#if defined(TONEDAC)
/****************************************/
/*            D A C						*/
/****************************************/

//DAC not available

#elif defined(TONEAC)

/****************************************/
/*            P  W  M	 Push-Pull				*/
/****************************************/

// ToneAC use 2 pin for PWM -3.3V to +3.3V

/***********************************/
void ToneHalAC_promini::init(void) {
/***********************************/
}

/***********************************/
void ToneHalAC_promini::init(uint32_t pin) {
/***********************************/
}

/***********************************/
void ToneHalAC_promini::init(uint32_t pin1, uint32_t pin2) {
/***********************************/
}

/***********************************/
void ToneHalAC_promini::tone(unsigned long frequency)
/***********************************/           
{
#if defined (TONEHAL_EXTENDED_VOLUME)	
	if (_toneMuted) return;
#if defined (TONEAC_LENGTH)
  toneAC(frequency,_volume, 512);
#elif defined(TONEAC_VOLUME)
  toneAC(frequency,_volume);
#else
	toneAc(frequency);
#endif		
#else
	toneAc(frequency);
#endif	
}

/***********************************/
void ToneHalAC_promini::tone(unsigned long frequency , uint8_t volume)
/***********************************/           
{
#if defined (TONEHAL_EXTENDED_VOLUME)	
	if (_toneMuted) return;
	if (volume > 10) volume = 10;
	_volume = volume;
#endif
#ifdef TONEAC_VOLUME
	toneAC(frequency, volume);
#else
	toneAC(frequency);
#endif		
}

/***********************************/
void ToneHalAC_promini::tone(unsigned long frequency , uint8_t volume, unsigned long length)
/***********************************/           
{
#if defined (TONEHAL_EXTENDED_VOLUME)	
	if (_toneMuted) return;
	if (volume > 10) volume = 10;
	_volume = volume;
	if (length > 1024) length = 1024;
#endif
	
#ifdef TONEAC_LENGTH
	toneAC(frequency, volume, length);
#elseif defined TONEAC_VOLUME
	toneAC(frequency, volume);
#else
	toneAC(frequency);
#endif	
}

/***********************************/
void ToneHalAC_promini::tone(unsigned long frequency , uint8_t volume, unsigned long length, uint8_t background)
/***********************************/           
{
#if defined (TONEHAL_EXTENDED_VOLUME)	
	if (_toneMuted) return;
	if (volume > 10) volume = 10;
	_volume = volume;
	if (length > 1024) length = 1024;
#endif
#ifdef TONEAC_LENGTH
	toneAC(frequency, volume, length, background);
#elseif defined TONEAC_VOLUME
	toneAC(frequency, volume);
#else
	toneAC(frequency);
#endif		
}

/***********************************/
void ToneHalAC_promini::noTone(void)
/***********************************/           
{
	noToneAC();
}

#elif defined(TONE)
/****************************************/
/*                P W M               	*/
/****************************************/

#elif defined(TONEI2S)
/****************************************/
/*            I  2  S					*/
/****************************************/

//I2S not available

#endif //Interface type
#endif //ARDUINO_AVR_PRO