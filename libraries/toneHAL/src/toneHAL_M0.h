/* toneHAL -- derived class for MKZERO - SAMD21
 *
 * Copyright 2019 Jean-philippe GOI
 * 
 * This file is part of ToneHAL.
 *
 * tonehal is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * tonehal is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
 
#ifndef toneHAL_M0_h
#define toneHAL_M0_h

#if defined(ARDUINO_ARCH_SAMD)


#include <toneHAL.h>

#if defined(TONEDAC)
#include <toneDAC_zero.h>

class ToneHalDAC_Zero : public ToneHal {
	public:

		void init(void);
		void init(uint32_t pin);
		void init(uint32_t pin1, uint32_t pin2);
		void tone(unsigned long frequency);
		void tone(unsigned long frequency, uint8_t volume);
		void tone(unsigned long frequency, uint8_t volume, unsigned long length);
		void tone(unsigned long frequency, uint8_t volume, unsigned long length, uint8_t background);

		void noTone();

		void setWaveForm(uint8_t form);

#if defined(TONEDAC_EXTENDED)
		void beginPlayWav(uint32_t srate);
		void endPlayWav();
		void playWav(const char *fname) ;
		bool isPlayingWav();
		uint32_t duration();
		uint32_t remaining();	
#endif //TONEDAC_EXTENDED
		
	private:
		ToneDacZero privateToneDacZero;
};	
	
#define ToneHAL ToneHalDAC_Zero	
	
#elif defined(TONEAC)
#include <toneAC_zero.h>

class ToneHalAC_Zero : public ToneHal {
	public:

		void init(void);
		void init(uint32_t pin);
		void init(uint32_t pin1, uint32_t pin2);
		void tone(unsigned long frequency);
		void tone(unsigned long frequency, uint8_t volume);
		void tone(unsigned long frequency, uint8_t volume, unsigned long length);
		void tone(unsigned long frequency, uint8_t volume, unsigned long length, uint8_t background);

		void noTone();

	private:
		ToneAcZero	privateToneACZero;
};	

#define ToneHAL ToneHalAC_Zero	

#elif defined(TONE)
#include <tone_zero.h>

class ToneHal_Zero : public ToneHal {
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
		ToneZero privateToneZero;
};	

#define ToneHAL ToneHal_Zero	

#elif defined(TONEI2S)

class ToneHalI2S_Zero : ToneHal{
	public:

		void init(void);
		void init(uint32_t pin);
		void init(uint32_t pin1, uint32_t pin2);
		void tone(unsigned long frequency);
		void tone(unsigned long frequency, uint8_t volume);
		void tone(unsigned long frequency, uint8_t volume, unsigned long length);
		void tone(unsigned long frequency, uint8_t volume, unsigned long length, uint8_t background);

		void noTone();

		void setWaveForm(uint8_t form);

		void beginPlayWav(uint32_t srate);
		void endPlayWav();
		void playWav(const char *fname) ;
		bool isPlayingWav();
		
		uint32_t duration();
		uint32_t remaining();		
};	

#define ToneHAL ToneHalI2S_Zero	

#endif //Interface type

#endif //ARDUINO_ARCH_SAMD

#endif

