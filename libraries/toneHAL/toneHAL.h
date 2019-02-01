#ifndef toneHAL_h
#define toneHAL_h

#include <Arduino.h>


/*********************************************************************************/
/*                                                                               */
/*                           Libraries ToneHal                                   */
/*                                                                               */
/*  version    Date     Description                                              */
/*    1.0    20/01/19                                                            */
/*    1.1    24/01/19   Réecriture des classes                                   */
/*                      répartition en plusieurs fichiers                        */
/*    1.2    26/01/19   Modifications mineures                                   */
/*                                                                               */
/*********************************************************************************/


#define WAVEFORM_SINUS			1
#define WAVEFORM_TRIANGLE		2
#define WAVEFORM_SQUARE			3
#define WAVEFORM_WAV 				4

	
#define DEFAULT_VOLUME 			5

/****************************/
/* Interface Type           */
/****************************/

//#define TONEI2S
//#define	TONEDAC
#define	TONEAC
//#define TONE

class ToneHal  {

	public:

		virtual void init(void) = 0;
		virtual void init(uint32_t pin) = 0;
    virtual void init(uint32_t pin1, uint32_t pin2) = 0;
    virtual void tone(unsigned long frequency) = 0;
    virtual void tone(unsigned long frequency, uint8_t volume) = 0;
    virtual void tone(unsigned long frequency, uint8_t volume, unsigned long length) = 0;
    virtual void tone(unsigned long frequency, uint8_t volume, unsigned long length, uint8_t background) = 0;

		virtual void noTone() = 0;

		//Volume 
		void setVolume(uint8_t newVolume = DEFAULT_VOLUME);
    uint8_t getVolume();

		    /* mute/unmute setting */
    void mute(bool newMuteState);

	protected:
		uint8_t _volume = 10;	
		bool 		_toneMuted = false;

};

#endif
	
	


