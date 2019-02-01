#include "toneHAL.h"
#include "toneHAL_PRO.h"
#include <Arduino.h>

#if defined(TONEDAC)
#elif defined(TONEAC)

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
	if (_toneMuted) return;
#if defined (TONEAC_LENGTH)
  toneAC(frequency,_volume, 512);
#elif defined(TONEAC_VOLUME)
  toneAC(frequency,_volume);
#else
	toneAc(frequency);
#endif			
}

/***********************************/
void ToneHalAC_promini::tone(unsigned long frequency , uint8_t volume)
/***********************************/           
{
	if (_toneMuted) return;
	if (volume > 10) volume = 10;
	_volume = volume;
#ifdef TONEAC_VOLUME
	toneAC(frequency, _volume);
#else
	toneAC(frequency);
#endif		
}

/***********************************/
void ToneHalAC_promini::tone(unsigned long frequency , uint8_t volume, unsigned long length)
/***********************************/           
{
	if (_toneMuted) return;
	if (volume > 10) volume = 10;
	_volume = volume;
	if (length > 1024) length = 1024;
	
#ifdef TONEAC_LENGTH
	toneAC(frequency, _volume, length);
#elseif defined TONEAC_VOLUME
	toneAC(frequency, _volume);
#else
	toneAC(frequency);
#endif	
}

/***********************************/
void ToneHalAC_promini::tone(unsigned long frequency , uint8_t volume, unsigned long length, uint8_t background)
/***********************************/           
{
	if (_toneMuted) return;
	if (volume > 10) volume = 10;
	_volume = volume;
	if (length > 1024) length = 1024;
#ifdef TONEAC_LENGTH
	toneAC(frequency, _volume, length, background);
#elseif defined TONEAC_VOLUME
	toneAC(frequency, _volume);
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
#elif defined(TONEI2S)
#endif //Interface type