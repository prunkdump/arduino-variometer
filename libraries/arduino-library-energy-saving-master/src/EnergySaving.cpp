#include "EnergySaving.h"

void EnergySaving::begin(){
	#ifdef ARDUINO_SAMD_TIAN
	pinMode(MIPS_PIN, OUTPUT);
	#endif
}

void EnergySaving::begin(unsigned int mode, unsigned int inter_pin, voidFuncPtr callback)
{

	begin();
	if((mode == WAKE_EXT_INTERRUPT) && (inter_pin !=2) &&  (inter_pin!=0) && (inter_pin!=1))
	{
		 NVMCTRL->CTRLB.bit.SLEEPPRM = 3;

		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
		pinMode(inter_pin,INPUT_PULLDOWN);
//		pinMode(inter_pin,INPUT_PULLUP);

//		attachInterrupt(inter_pin,callback, CHANGE);
		attachInterrupt(inter_pin,callback, RISING);
		enable_eic_wake(inter_pin);
		set_clk();

	}
	else return;
}


void EnergySaving::begin(unsigned int mode)
{
	begin();

	if(mode == WAKE_RTC_ALARM)
	{
		//RTCInt.begin(TIME_H24);

		GCLK->CLKCTRL.bit.CLKEN = 0; //disable GCLK module
		while (GCLK->STATUS.bit.SYNCBUSY);

		GCLK->GENCTRL.bit.RUNSTDBY = 1;  //GCLK6 run standby
		while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

		GCLK->CLKCTRL.bit.CLKEN = 1; //disable GCLK module
		while (GCLK->STATUS.bit.SYNCBUSY);

		NVMCTRL->CTRLB.bit.SLEEPPRM = 3;

		SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

	}
	else return;

}


void EnergySaving::standby(void)
{
	__DSB();
	__WFI();

}


void EnergySaving::set_clk(void)
{
	GCLK->CLKCTRL.bit.CLKEN = 0; //disable GCLK module
		while (GCLK->STATUS.bit.SYNCBUSY);

		GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK6 | GCLK_CLKCTRL_ID( GCM_EIC )) ;  //EIC clock switched on GCLK6
		while (GCLK->STATUS.bit.SYNCBUSY);

		GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_ID(6));  //source for GCLK6 is OSCULP32K
		while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

		GCLK->GENCTRL.bit.RUNSTDBY = 1;  //GCLK6 run standby
		while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

}


void EnergySaving::enable_eic_wake(unsigned int inter_pin)
{
	switch(inter_pin)
		{
			case 3:
				EIC->WAKEUP.bit.WAKEUPEN9 = 1;
				break;
			case 4:
				EIC->WAKEUP.bit.WAKEUPEN14 = 1;
				break;
			case 5:
				EIC->WAKEUP.bit.WAKEUPEN15 = 1;
				break;
			case 6:
				//EIC->WAKEUP.bit.WAKEUPEN20 = 1;  //non si può usare
				break;
			case 7:
				//EIC->WAKEUP.bit.WAKEUPEN21 = 1;  //non si può usare
				break;
			case 8:
				EIC->WAKEUP.bit.WAKEUPEN6 = 1;
				break;
			case 9:
				EIC->WAKEUP.bit.WAKEUPEN7 = 1;
				break;
			case 10:
				//EIC->WAKEUP.bit.WAKEUPEN18 = 1;  //non si può usare
				break;
			case 11:
				//EIC->WAKEUP.bit.WAKEUPEN16 = 1;  //non c'è
				break;
			case 12:
				//EIC->WAKEUP.bit.WAKEUPEN19 = 1;  //non si può usare
				break;
			case 13:
				//EIC->WAKEUP.bit.WAKEUPEN17 = 1;  //non c'è
				break;
			default:
				break;
		}
}

#ifdef ARDUINO_SAMD_TIAN

 EnergySaving::begin(){
 	pinMode(MIPS_PIN, OUTPUT);
 }

void EnergySaving::maxLowPowerMode()
{
	//Energy Saving Tian OFF
	digitalWrite(MIPS_PIN, LOW);
}

void EnergySaving::noLowPowerMode()
{
	//Energy Saving Tian ON
	digitalWrite(MIPS_PIN, HIGH);
}

#endif
