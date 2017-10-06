/********************************************************************************************
 This sketch demonstrates how sleep function using external interrupt to wake up the MCU.
 This sketch work only cortex-m0+ boards.
*********************************************************************************************/



#include <EnergySaving.h>

#define VARIO_PIN_RST    0

EnergySaving nrgSave;

unsigned int i=0;

void setup()
{

  digitalWrite(VARIO_PIN_RST, HIGH);   // Hard Reset M0 )
  pinMode(VARIO_PIN_RST, OUTPUT);

  nrgSave.begin(WAKE_EXT_INTERRUPT, 6, dummy);  //standby setup for external interrupts
}

void loop()
{
  for(i=0; i<20; i++)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN,LOW);
    delay(500);
  }

  nrgSave.standby();  //now mcu goes in standby mode

//  NVIC_SystemReset();      // processor software reset 

   digitalWrite(VARIO_PIN_RST, LOW);   // Hard Reset M0 )
   delay(500);

  digitalWrite(LED_BUILTIN,HIGH);
  delay(2000);
  digitalWrite(LED_BUILTIN,LOW);
  delay(1000);

}


void dummy(void)  //interrupt routine (isn't necessary to execute any tasks in this routine
{
   // digitalWrite(LED_BUILTIN,HIGH);
}
