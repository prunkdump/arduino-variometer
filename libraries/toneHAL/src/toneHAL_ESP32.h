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
	
#elif defined(TONEPWM)
#include <toneAC_esp32.h>

class ToneHalPWM_Esp32 : public ToneHal {
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
		ToneAcEsp32	privateTonePWMEsp32;
};	

#define ToneHAL ToneHalPWM_Esp32	

#elif defined(TONE)
#include <tone32.h>

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

