#ifndef toneHAL_PRO_h
#define toneHAL_PRO_h

#if defined(ARDUINO_AVR_PRO)

#include <toneHAL.h>

#if defined(TONEDAC)
//DAC not available
#elif defined(TONEPWM)
// ToneAC use 2 pin for PWM -3.3V to +3.3V
#include "toneAC.h"

class ToneHalPWM_promini : public ToneHal {
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

#define ToneHAL ToneHalPWM_promini	

#elif defined(TONE)

#elif defined(TONEI2S)
//I2S not available
#endif //Interface type
#endif //ARDUINO_AVR_PRO

#endif
