/* toneHAL -- derived class for ProMini
 *
 * Copyright 2019 Jean-philippe GOI
 * 
 * This file is part of ToneHAL.
 *
 * toneHAL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * toneHAL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

 #ifndef toneHAL_PRO_h
#define toneHAL_PRO_h

#if defined(ARDUINO_AVR_PRO)

#include <toneHAL.h>

#if defined(TONEDAC)
//DAC not available
#elif defined(TONEAC)
// ToneAC use 2 pin for PWM -3.3V to +3.3V
#include "toneAC.h"

class ToneHalAC_promini : public ToneHal {
	public:

		void init(void);
		void init(uint32_t pin);
		void init(uint32_t pin1, uint32_t pin2);
		void tone(unsigned long frequency);
		void tone(unsigned long frequency, uint8_t volume);
		void tone(unsigned long frequency, uint8_t volume, unsigned long length);
		void tone(unsigned long frequency, uint8_t volume, unsigned long length, uint8_t background);

		void noTone();
		
	protected:
		uint32_t _pin;
};	

#define ToneHAL ToneHalAC_promini	

#elif defined(TONE)

#elif defined(TONEI2S)
//I2S not available
#endif //Interface type
#endif //ARDUINO_AVR_PRO

#endif
