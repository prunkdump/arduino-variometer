#include <Wire.h>
#include <SDP6xx.h>

void setup()
{
  Wire.begin(); 
  Serial.begin(115200);
}


void loop()
{
   Serial.println((int)SDP6xx.readPA());
   delay(500);
}
