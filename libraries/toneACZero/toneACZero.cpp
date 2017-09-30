#include <Arduino.h>
#include <toneACZero.h>

uint32_t _tAC_volume[] = { 200, 100, 67, 50, 40, 33, 29, 22, 11, 2 };
uint32_t _tAC2_volume[] = { 150, 72, 51, 38, 32, 23, 20, 19, 10, 2 };

/* route GCLK4 Clock to TCC0 */ 
void toneAC_initClock(void) {

  /*****************/
  /* Generic Clock */
  /*****************/

  /* set divisor */
  GCLK->GENDIV.reg = GCLK_GENDIV_DIV(1) | GCLK_GENDIV_ID(4);
  while(GCLK->STATUS.bit.SYNCBUSY);

  /* set generator */
  GCLK->GENCTRL.reg = GCLK_GENCTRL_IDC | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL48M | GCLK_GENCTRL_ID(4);
  while(GCLK->STATUS.bit.SYNCBUSY);

  /* assign clock to timer TCC0 TCC1 */
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN(4) | GCLK_CLKCTRL_ID_TCC0_TCC1;
  while(GCLK->STATUS.bit.SYNCBUSY);
}


/* !!! Before, a Generic Clock must be routed to TCC0 !!! */
void toneAC_init(void) {
  
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

void toneAC_notone(void) {
  
  /* disable multiplexing */
  PORT->Group[0].PINCFG[4].reg = 0;
  PORT->Group[0].PINCFG[5].reg = 0;

  /* stop timer */
  TCC0->CTRLA.bit.ENABLE = 0;
  while (TCC0->SYNCBUSY.bit.ENABLE);    
}


void toneAC1(uint32_t frequency, unsigned volume) {
  
  /****************/
  /* check volume */
  /****************/
  if(frequency == 0 || volume == 0) {
    toneAC_notone();
    return;
  }
  
  if(volume > 10) {
    volume = 10;
  }
  
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
  
  /* set counter compare value for WO[0] and WO[1] */
  top /= _tAC_volume[volume - 1];
  TCC0->CC[0].reg = top;
  while (TCC0->SYNCBUSY.bit.CC0);
  
  TCC0->CC[1].reg = top;
  while (TCC0->SYNCBUSY.bit.CC1);
  
  /* enable timer */
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_ENABLE;
  while (TCC0->SYNCBUSY.bit.ENABLE);  
}


void toneAC2(uint32_t frequency, unsigned volume) {
  
  /****************/
  /* check volume */
  /****************/
  if(frequency == 0 || volume == 0) {
    toneAC_notone();
    return;
  }
  
  if(volume > 10) {
    volume = 10;
  }
  
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
  uint32_t top = F_CPU / frequency / 2;
  
  /* set TOP */
  TCC0->PER.reg = top;
  while (TCC0->SYNCBUSY.bit.PER); 
  
  /* set counter compare value for WO[0] and WO[1] */
  uint32_t cm0 = top / _tAC2_volume[volume - 1];
  uint32_t cm1 = top - cm0;  
  
  TCC0->CC[0].reg = cm0;
  while (TCC0->SYNCBUSY.bit.CC0);
  
  TCC0->CC[1].reg = cm1;
  while (TCC0->SYNCBUSY.bit.CC1);
  
  /* enable timer */
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_ENABLE;
  while (TCC0->SYNCBUSY.bit.ENABLE);  
}







  


  
  
