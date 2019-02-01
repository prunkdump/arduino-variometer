#ifndef toneHAL_PRO_h
#define toneHAL_PRO_h

#include <toneHAL.h>

#if defined(TONEDAC)

#elif defined(TONEAC)
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

#endif //Interface type

#endif
