// ---------------------------------------------------------------------------
// Created by Tim Eckel - teckel@leethost.com
// Copyright 2013 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
//
// See "toneAC.h" for purpose, syntax, version history, links, and more.
// ---------------------------------------------------------------------------

#include "toneAC_zero.h"
#include <GenClock_zero.h>

static bool toneACMuted = false;

#ifdef TONEAC_LENGTH
unsigned long _tAC_time; // Used to track end note with timer when playing note in the background.
#endif

#ifdef TONEAC_VOLUME
//uint8_t _tAC_volume[] = { 200, 100, 67, 50, 40, 33, 29, 22, 11, 2 }; // Duty for linear volume control.
uint8_t _tAC_volume[] = { 150, 72, 51, 38, 32, 23, 20, 19, 10, 2 }; //new duty values for three phased Low Power mode
#endif


void toneAC_init(void) {

  /* need generic clock */
  initGenClock();

  /********/
  /* PORT */
  /********/

  /* set as OUTPUT */
  PORT->Group[0].DIRSET.reg = PORT_PA04;
  PORT->Group[0].DIRSET.reg = PORT_PA05;

  /* put LOW */
  PORT->Group[0].OUTCLR.reg = PORT_PA04;
  PORT->Group[0].OUTCLR.reg = PORT_PA05;

  /* set multiplexing with function E */
  //PORT->Group[0].PMUX[4 >> 1].reg |= PORT_PMUX_PMUXE_E;
  //PORT->Group[0].PMUX[5 >> 1].reg |= PORT_PMUX_PMUXO_E;
  PORT->Group[0].PMUX[4 >> 1].reg = PORT_PMUX_PMUXE_E | PORT_PMUX_PMUXO_E;

  /* disable all pins functionnalities */
  PORT->Group[0].PINCFG[4].reg = 0;
  PORT->Group[0].PINCFG[5].reg = 0;

  /********/
  /* TCC0 */
  /********/

  /* set wave and polarity (inverted) for WO[0] and WO[1] */
  TCC0->WAVE.reg = TCC_WAVE_POL(0x01) | TCC_WAVE_WAVEGEN_DSBOTH;
  while (TCC0->SYNCBUSY.bit.WAVE);
}


void toneAC(unsigned long frequency
#ifdef TONEAC_VOLUME
            , uint8_t volume
#endif
#ifdef TONEAC_LENGTH
            , unsigned long length, uint8_t background
#endif
	    ) {

  /* check if no tone */ 
  if (toneACMuted || frequency == 0
#ifdef TONEAC_VOLUME     
      || volume == 0
#endif
      ) { noToneAC(); return; } 

  /* check volume */
#ifdef TONEAC_VOLUME
  if (volume > 10) volume = 10;
#endif

  /********/
  /* PORT */
  /********/

  /* enable multiplexing */
  PORT->Group[0].PINCFG[4].bit.PMUXEN = 1;
  PORT->Group[0].PINCFG[5].bit.PMUXEN = 1;

  /********/
  /* TCC0 */
  /********/

  /* compute top */
  uint32_t top = F_CPU / frequency / 2 - 1;

  /* set TOP */
  TCC0->PER.reg = top;
  while (TCC0->SYNCBUSY.bit.PER);

  /* set duty cycle */
#ifdef TONEAC_VOLUME
  uint32_t duty = top / _tAC_volume[volume - 1]; // Calculate the duty cycle (volume).
#else
  uint32_t duty = top >> 1;
#endif

  /* set counter compare value for WO[0] and WO[1] */
  TCC0->CC[0].reg = duty;
  while (TCC0->SYNCBUSY.bit.CC0);

  TCC0->CC[1].reg = top - duty;
  while (TCC0->SYNCBUSY.bit.CC1);

  /* enable timer */
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_ENABLE;
  while (TCC0->SYNCBUSY.bit.ENABLE);

  /* compute length time */
#ifdef TONEAC_LENGTH
  if (length > 0 && background) {  // Background tone playing, returns control to your sketch.

    _tAC_time = millis() + length; // Set when the note should end.
    //TODO !!!                     // Activate the timer interrupt.
  }

  if (length > 0 && !background) { delay(length); noToneAC(); } // Just a simple delay, doesn't return control untill finished.
#endif
}

void noToneAC() {
#ifdef TONEAC_LENGTH
  //TODO !!!!                     // Remove the timer interrupt.
#endif
  /* disable multiplexing */
  PORT->Group[0].PINCFG[4].reg = 0;
  PORT->Group[0].PINCFG[5].reg = 0;

  /* stop timer */
  TCC0->CTRLA.bit.ENABLE = 0;
  while (TCC0->SYNCBUSY.bit.ENABLE);
}

#ifdef TONEAC_LENGTH
// TODO !!! { // Timer interrupt vector.
//  if (millis() >= _tAC_time) noToneAC(); // Check to see if it's time for the note to end.
//}
#endif

void toneACMute(bool newMuteState) {

  /* stop tone if needed */
  if( newMuteState ) {
    noToneAC();
  }

  /* save */
  toneACMuted = newMuteState;
}


