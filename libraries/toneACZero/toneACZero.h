#ifndef TONE_AC_ZERO_H
#define TONE_AC_ZERO_H

#include <Arduino.h>

/* route GCLK4 Clock to TCC0 */ 
void toneAC_initClock(void);

/*  INIT                                                       */
/* !!! Before init, a Generic Clock must be routed to TCC0 !!! */
void toneAC_init(void);

/* you can also use toneAC(0) */
void toneAC_notone(void);

/* tone */
void toneAC1(uint32_t frequency, unsigned volume = 10);
void toneAC2(uint32_t frequency, unsigned volume = 10);

#define toneAC toneAC2

#endif
