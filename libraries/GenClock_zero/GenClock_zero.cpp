#include <GenClock_zero.h>

#include <Arduino.h>

static bool genClockStarted = false;

/* route GCLK4 Clock to TCC0/TCC1 */
void initGenClock(void) {

  if(!genClockStarted) {

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

    genClockStarted = true;
  }
}
