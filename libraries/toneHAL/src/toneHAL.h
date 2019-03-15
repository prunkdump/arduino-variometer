/* toneHAL -- tone HAL
 *
 * Copyright 2019 Jean-philippe GOI
 * 
 * This file is part of GNUVario.
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
 
/*
 * \par Copyright (C), 2016-2017, M5Stack
 * \class ToneHal
 * \brief   ToneHal library.
 * @file    ToneHal.h
 * @author  JPG63
 * @version V1.4.0
 * @date    2019/03/09
 * @brief   Header for ToneHal.cpp module
 *
 * \par Description
 * 
 *
 * \par Method List:
 *    
 *  System:
        ToneHal.init();
				ToneHal.init(uint32_t pin);
				ToneHal.init(uint32_t pin1, uint32_t pin2);

    Tone:
				ToneHal.tone(unsigned long frequency);
				ToneHal.tone(unsigned long frequency, uint8_t volume);
				ToneHal.tone(unsigned long frequency, uint8_t volume, unsigned long length);
				ToneHal.tone(unsigned long frequency, uint8_t volume, unsigned long length, uint8_t background);

				ToneHal.noTone();

		TONEHAL_EXTENDED_VOLUME
		
				ToneHal.setVolume(uint8_t newVolume = DEFAULT_VOLUME);
				ToneHal.getVolume();
				ToneHal.mute(bool newMuteState);


 *
 * \par History:
 * <pre>
 * `<Author>`         `<Time>`        `<Version>`        `<Descr>`
 *                     2019/01/19        1.0.0          Rebuild the new.
 *                     2019/03/12        1.4.1          minor changes
 * </pre>
 *
 */

/***********************************************************************************/
/*                                                                                 */
/*                           Libraries ToneHal                                     */
/*                                                                                 */
/*  version    Date     Description                                                */
/*    1.0.0    20/01/19                                                            */
/*    1.1.0    24/01/19   Réecriture des classes                                   */
/*                        répartition en plusieurs fichiers                        */
/*    1.1.1    26/01/19   Modifications mineures                                   */
/*    1.3.0    09/02/19   Ajout TONEHAL_EXTENDED_VOLUME							   */
/*    1.4.0    02/03/19   Réécriture de la bibliotheque                            */
/*                        Ajout ESP32                                              */
/*    1.4.1    12/03/19   Modifications mineures								   */
/*                                                                                 */
/***********************************************************************************/

 
#ifndef toneHAL_h
#define toneHAL_h

#include <Arduino.h>

#define WAVEFORM_SINUS			1
#define WAVEFORM_TRIANGLE		2
#define WAVEFORM_SQUARE			3
#define WAVEFORM_WAV 			4

	
#define DEFAULT_VOLUME 			5

#define TONEHAL_EXTENDED_VOLUME

/****************************/
/* Interface Type           */
/****************************/

//#define TONEI2S
//#define	TONEDAC
#define	TONEPWM
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

#if defined (TONEHAL_EXTENDED_VOLUME)
		
		//Volume 
		void setVolume(uint8_t newVolume = DEFAULT_VOLUME);
    uint8_t getVolume();

		    /* mute/unmute setting */
    void mute(bool newMuteState);

	protected:
		uint8_t _volume = 10;	
		bool 		_toneMuted = false;

#endif //TONEHAL_EXTENDED_VOLUME		
};

#endif
	
	


