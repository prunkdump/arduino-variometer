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
	
#elif defined(TONEPWM)
#include <toneAC_zero.h>

class ToneHalPWM_Zero : public ToneHal {
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
		ToneAcZero	privateTonePWMZero;
};	

#define ToneHAL ToneHalPWM_Zero	

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

