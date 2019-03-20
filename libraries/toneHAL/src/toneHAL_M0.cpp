/* toneHAL -- derived class for MKZERO
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
#include "toneHAL_M0.h"

#if defined(ARDUINO_ARCH_SAMD)
#include <Arduino.h>
#include "Debug.h"
//#include "utility\Debug.h"

#define TONEDAC_VOLUME  //set to have volume control
//#define TONEDAC_LENGTH  //set to have length control

#if defined(TONEDAC)
/****************************************/
/*            D A C						*/
/****************************************/

/***********************************/
void ToneHalDAC_Zero::init(void) {
/***********************************/
  privateToneDacZero.init();
}

/***********************************/
void ToneHalDAC_Zero::init(uint32_t pin) {
/***********************************/
  privateToneDacZero.init();
}

/***********************************/
void ToneHalDAC_Zero::init(uint32_t pin1, uint32_t pin2) {
/***********************************/
  privateToneDacZero.init();
}

/***********************************/
void ToneHalDAC_Zero::tone(unsigned long frequency)
/***********************************/           
{
#if defined (TONEHAL_EXTENDED_VOLUME)
	if (_toneMuted) return;
  privateToneDacZero.tone(frequency, _volume*10);
#else
	privateToneDacZero.tone(frequency, privateToneDacZero.getVolume());
#endif	
}

/***********************************/
void ToneHalDAC_Zero::tone(unsigned long frequency , uint8_t volume)
/***********************************/           
{
#if defined (TONEHAL_EXTENDED_VOLUME)
	if (_toneMuted) return;
	if (volume > 10) volume = 10;
	_volume = volume;
#endif
	
#ifdef TONEDAC_DEBUG	
	SerialPort.print("volume toneHalDac = ");
	SerialPort.println(volume);
#endif TONEDAC_DEBUG
	
#ifdef TONEDAC_DEBUG	
	SerialPort.println("privateToneDacZero(freq,vol)");
#endif TONEDAC_DEBUG
  privateToneDacZero.tone(frequency, volume*10);
}

/***********************************/
void ToneHalDAC_Zero::tone(unsigned long frequency , uint8_t volume, unsigned long length)
/***********************************/           
{
#if defined (TONEHAL_EXTENDED_VOLUME)
	if (_toneMuted) return;
	if (volume > 10) volume = 10;
	_volume = volume;
#endif
  privateToneDacZero.tone(frequency, volume*10);
}

/***********************************/
void ToneHalDAC_Zero::tone(unsigned long frequency , uint8_t volume, unsigned long length, uint8_t background)
/***********************************/
{
#if defined (TONEHAL_EXTENDED_VOLUME)
	if (_toneMuted) return;
	if (volume > 10) volume = 10;
	_volume = volume;
#endif

	tone(frequency, volume*10, length);
}

/***********************************/
void ToneHalDAC_Zero::noTone() {
/***********************************/
  privateToneDacZero.noTone();
}

/***********************************/
void ToneHalDAC_Zero::setWaveForm(uint8_t form) {
/***********************************/
  privateToneDacZero.setWaveForm(form);
}
		
#if defined(TONEDAC_EXTENDED)
/***********************************/
void ToneHalDAC_Zero::beginPlayWav(uint32_t srate)
/***********************************/           
{
	privateToneDacZero.begin(srate);
}

/***********************************/
void ToneHalDAC_Zero::endPlayWav()
/***********************************/          
{
	privateToneDacZero.end();
}

/***********************************/
void ToneHalDAC_Zero::playWav(const char *fname) {
/***********************************/
  privateToneDacZero.play(fname);
}

/***********************************/
bool ToneHalDAC_Zero::isPlayingWav() {
/***********************************/
  return privateToneDacZero.isPlaying();
}

/***********************************/
uint32_t ToneHalDAC_Zero::duration() {
/***********************************/
  return privateToneDacZero.duration();
}

/***********************************/
uint32_t ToneHalDAC_Zero::remaining() {
/***********************************/
  return privateToneDacZero.remaining();
}

#endif //TONEDAC_EXTENDED
#elif defined(TONEAC)
/****************************************/
/*            P  W  M					*/
/****************************************/

/***********************************/
void ToneHalAC_Zero::init(void) {
/***********************************/
	privateToneACZero.init();
}

/***********************************/
void ToneHalAC_Zero::init(uint32_t pin) {
/***********************************/
	privateToneACZero.init();
}

/***********************************/
void ToneHalAC_Zero::init(uint32_t pin1, uint32_t pin2) {
/***********************************/
	privateToneACZero.init();
}

/***********************************/
void ToneHalAC_Zero::tone(unsigned long frequency)
/***********************************/           
{
#if defined (TONEHAL_EXTENDED_VOLUME)
	if (_toneMuted) return;
  privateToneACZero.tone(frequency,_volume);	
#else
  privateToneACZero.tone(frequency, 10);	
#endif
}

/***********************************/
void ToneHalAC_Zero::tone(unsigned long frequency , uint8_t volume)
/***********************************/           
{
#if defined (TONEHAL_EXTENDED_VOLUME)	
	if (_toneMuted) return;
	if (volume > 10) volume = 10;
	_volume = volume;
#endif
#ifdef TONEAC_VOLUME
	privateToneACZero.tone(frequency, volume);
#else
	privateToneACZero.tone(frequency);
#endif		
}

/***********************************/
void ToneHalAC_Zero::tone(unsigned long frequency , uint8_t volume, unsigned long length)
/***********************************/           
{
#if defined (TONEHAL_EXTENDED_VOLUME)	
	if (_toneMuted) return;
	if (volume > 10) volume = 10;
	_volume = volume;
	if (length > 1024) length = 1024;
#endif
	
#ifdef TONEAC_LENGTH
		  privateToneACZero.tone(frequency, volume, length);
#else
#ifdef TONEAC_VOLUME
	privateToneACZero.tone(frequency, volume);
#else
	privateToneACZero.tone(frequency);
#endif	
#endif

}

/***********************************/
void ToneHalAC_Zero::tone(unsigned long frequency , uint8_t volume, unsigned long length, uint8_t background)
/***********************************/           
{
#if defined (TONEHAL_EXTENDED_VOLUME)	
	if (_toneMuted) return;
	if (volume > 10) volume = 10;
	_volume = volume;
	if (length > 1024) length = 1024;
#endif
#ifdef TONEAC_LENGTH
		  privateToneACZero.tone(frequency, volume, length, background);
#else
#ifdef TONEAC_VOLUME
	privateToneACZero.tone(frequency, volume);
#else
	privateToneACZero.tone(frequency);
#endif	
#endif	
}

/***********************************/
void ToneHalAC_Zero::noTone(void)
/***********************************/           
{
	privateToneACZero.noTone();
}


#elif defined(TONE)
/****************************************/
/*   Standard Library     T O N E   	*/
/****************************************/

/***********************************/
void ToneHal_Zero::init(void) {
/***********************************/
  _pin = 2;
}

/***********************************/
void ToneHal_Zero::init(uint32_t pin) {
/***********************************/
  _pin = pin;
}

/***********************************/
void ToneHal_Zero::init(uint32_t pin1, uint32_t pin2) {
/***********************************/
  _pin = pin1;
}

/***********************************/
void ToneHal_Zero::tone(unsigned long frequency)
/***********************************/           
{
	if (_toneMuted) return;
  privateToneZero.tone(_pin,frequency,512);	
}

/***********************************/
void ToneHal_Zero::tone(unsigned long frequency , uint8_t volume)
/***********************************/           
{
	if (_toneMuted) return;
	if (volume > 10) volume = 10;
	_volume = volume;
  privateToneZero.tone(_pin,frequency,512);
}

/***********************************/
void ToneHal_Zero::tone(unsigned long frequency , uint8_t volume, unsigned long length)
/***********************************/           
{
	if (_toneMuted) return;
	if (volume > 10) volume = 10;
	_volume = volume;
	if (length > 1024) length = 1024;
  privateToneZero.tone(_pin,frequency,length);
}

/***********************************/
void ToneHal_Zero::tone(unsigned long frequency , uint8_t volume, unsigned long length, uint8_t background)
/***********************************/
{
	tone(frequency, volume, length);
}

/***********************************/
void ToneHal_Zero::noTone(void)
/***********************************/           
{
	privateToneZero.noTone(_pin);
}

#elif defined(TONEI2S)
/****************************************/
/*            I  2  S					*/
/****************************************/

#endif //Interface type

#endif //ARDUINO_ARCH_SAMD