/***********************************************************************************************
 This sketch demonstrates how to use sleep features and wake-up MCU with RTC alarm interrupt.
 This sketch works only on cortex-m0+ based boards.
************************************************************************************************/

#include <EnergySaving.h>
#include <RTCInt.h>

EnergySaving nrgSave;
RTCInt rtc;

unsigned int i=0;

void setup()
{

  pinMode(30,OUTPUT);  //led RX
  Serial5.begin(9600);   //serial settings
  rtc.begin(TIME_H24);
  rtc.setTime(12,48,0,0);
  rtc.setDate(22,10,15);
  rtc.enableAlarm(SEC,ALARM_INTERRUPT,rest_alarm_int);
  rtc.local_time.hour=12;
  rtc.local_time.minute=48;
  rtc.local_time.second=30;
  nrgSave.begin(WAKE_RTC_ALARM);  //standby setup for external interrupts
  rtc.setAlarm();
  SerialUSB.println("START");

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

  digitalWrite(30,LOW);
  nrgSave.standby();  //now mcu go to standby

}

void rest_alarm_int(void)  //interrupt routine
{
  //PORT->Group[1].OUTSET.reg = 0x1u<<3;
  digitalWrite(30,HIGH);

  Serial5.println("HELLO");
}
