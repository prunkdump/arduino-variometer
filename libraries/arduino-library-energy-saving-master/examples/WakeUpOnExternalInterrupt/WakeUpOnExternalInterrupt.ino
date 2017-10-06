/********************************************************************************************
 This sketch demonstrates how sleep function using external interrupt to wake up the MCU.
 This sketch work only cortex-m0+ boards.
*********************************************************************************************/



#include <EnergySaving.h>

EnergySaving nrgSave;

unsigned int i=0;

void setup()
{
  nrgSave.begin(WAKE_EXT_INTERRUPT, 8, dummy);  //standby setup for external interrupts
}

void loop()
{
  for(i=0; i<20; i++)
  {
    digitalWrite(13,HIGH);
    delay(500);
    digitalWrite(13,LOW);
    delay(500);
  }

  nrgSave.standby();  //now mcu goes in standby mode
}


void dummy(void)  //interrupt routine (isn't necessary to execute any tasks in this routine
{

}
