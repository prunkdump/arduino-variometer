/*

This sketch allows the users to manage the Tian and MCU power management.
Users can power up or power down MIPS and MCU with an external interrupt.

*/

#include <EnergySaving.h>

EnergySaving nrgSave;


#define MIPS_PIN 32  //PA28 PIN 32

void setup()
{
  // pinMode(MIPS_PIN,OUTPUT);
  // digitalWrite(MIPS_PIN,HIGH);

	nrgSave.begin(WAKE_EXT_INTERRUPT, 8, wakeUp);  //standby setup for external interrupts
	nrgSave.noLowPowerMode();
}

void loop()
{
  doSomething();
  sleep();
}

void doSomething(){
  //blink for 90 seconds
  for(int i=0; i<90; i++)
  {
    digitalWrite(13,HIGH);
    delay(500);
    digitalWrite(13,LOW);
    delay(500);
  }
}

void sleep(void){
  //digitalWrite(MIPS_PIN, LOW);
	nrgSave.maxLowPowerMode();
  nrgSave.standby();  //now mcu goes in standby mode
}

void wakeUp(void)  //interrupt routine (isn't necessary to execute any tasks in this routine
{
  //mcu is waked-up by the interrupt

  //wake up mips
  //digitalWrite(MIPS_PIN,HIGH);
	nrgSave.noLowPowerMode();
}
