/* toneHAL -- derived class for ESP32
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
 
#ifndef toneHAL_ESP32_h
#define toneHAL_ESP32_h

#if defined(ESP32)

#include <toneHAL.h>

#if defined(TONEDAC)
#include <toneDAC_esp32.h>

class ToneHalDAC_Esp32 : public ToneHal {
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
		ToneDacEsp32 privateToneDacEsp32;
};	
	
#define ToneHAL ToneHalDAC_Esp32	
	
#elif defined(TONEAC)

#elif defined(TONE)
//#include <tone32.h>
#include <tone_esp32.h>

class ToneHal_Esp32 : public ToneHal {
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
		ToneEsp32 privateToneEsp32;
};	

#define ToneHAL ToneHal_Esp32	

#elif defined(TONEI2S)

class ToneHalI2S_Esp32 : ToneHal{
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

#define ToneHAL ToneHalI2S_Esp32	

#endif //Interface type

#endif //ESP32

#endif

