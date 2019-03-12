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

#elif defined(TONEPWM)

/****************************************/
/*            P  W  M					*/
/****************************************/

// ToneAC use 2 pin for PWM -3.3V to +3.3V

/***********************************/
void ToneHalPWM_promini::init(void) {
/***********************************/
}

/***********************************/
void ToneHalPWM_promini::init(uint32_t pin) {
/***********************************/
}

/***********************************/
void ToneHalPWM_promini::init(uint32_t pin1, uint32_t pin2) {
/***********************************/
}

/***********************************/
void ToneHalPWM_promini::tone(unsigned long frequency)
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
void ToneHalPWM_promini::tone(unsigned long frequency , uint8_t volume)
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
void ToneHalPWM_promini::tone(unsigned long frequency , uint8_t volume, unsigned long length)
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
void ToneHalPWM_promini::tone(unsigned long frequency , uint8_t volume, unsigned long length, uint8_t background)
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
void ToneHalPWM_promini::noTone(void)
/***********************************/           
{
	noToneAC();
}

#elif defined(TONE)
/****************************************/
/*   Standard Library     T O N E   	*/
/****************************************/

#elif defined(TONEI2S)
/****************************************/
/*            I  2  S					*/
/****************************************/

//I2S not available

#endif //Interface type
#endif //ARDUINO_AVR_PRO