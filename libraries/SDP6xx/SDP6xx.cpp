/*
SDP6xx - Library for the SDP6xx series of Sensirion sensors.

Copyright 2012 Eduard Iten

Supported devices:
SHT20*
SHT21
SHT25
SDP610

*The SHT20 has not been tested so far, but should work according
the Sensirion datasheet. If anyone can confirm this, please report.

This library is free software, it is released under the GNU GPL v3.
Please find the terms and conditions in the attached gpl.txt or
in the world wide web: <http://www.gnu.org/licenses/>

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
or check in the web: <http://www.gnu.org/licenses/>
*/

#include "SDP6xx.h"

uint32_t timeout=0;

void SDP6xxClass::softReset ()//new: remove Class
{
     Wire.beginTransmission(SDP6xxADDR);
     Wire.write(SOFT_RESET);
     delay(15);
}

uint8_t SDP6xxClass::readUserRegister()//new: remove Class

{
     Wire.beginTransmission(SDP6xxADDR);
     Wire.write(USER_REG_R);
     Wire.endTransmission();
     Wire.requestFrom(SDP6xxADDR,(uint8_t)2);
     while(Wire.available()<2) {
          ;
     }
     return Wire.read();
}

void SDP6xxClass::writeUserRegister(uint8_t userRegister)//new: remove Class

{
     Wire.beginTransmission(SDP6xxADDR);
     Wire.write(userRegister);
     Wire.endTransmission();
}


uint16_t SDP6xxClass::readMeasurement(SDP6xxMeasureType type)//new: remove Class

{
//     uint16_t value = 0;
int value = 0;
     uint8_t low, high;
int high16 = 0;

     Wire.beginTransmission(SDP6xxADDR);
//Serial.print(" W.beginSDPadr: ");
//Serial.print(SDP6xxADDR, HEX);
     switch (type) {
     case PRESSURE:
	    Wire.write(MEASUREMENT_PA_HM);
//Serial.print(" Meas-PA-HM: ");
//Serial.print(MEASUREMENT_PA_HM, HEX);
	    break;
     case HUMIDITY:
          Wire.write(MEASUREMENT_RH_HM);
          break;
     case TEMP:
          Wire.write(MEASUREMENT_T_HM);
          break;
     }
     Wire.endTransmission();
//Serial.print(" Wire-end ");
delay(100); //new; replacing "digitalRead(18)", which is not working on MEGA
     /*wait for measurement to complete.
     timeout= millis()+300;
     while (!digitalRead(18)) {
          if (millis()>timeout) {
               return 0;
          }
     } //end wait digitalread
*/ 
//Serial.print(" wire.Request ");
     Wire.requestFrom(SDP6xxADDR,(uint8_t)3);
//Serial.print(" SDP-ADDR: "), Serial.print(SDP6xxADDR, HEX);
     timeout=millis()+300;
     while (Wire.available()<3) {
          if (millis()>timeout) {
               return 0;
          }
     }
     high=Wire.read();
     low=Wire.read();
/*
Serial.print(" high:");
Serial.print(high);
Serial.print(" low:");
Serial.print(low); 
*/
high16 = (int)high;
//Serial.print(" u16High: ");
//Serial.print(high16);
value = high16 << 8 | low;
//     value=(uint16_t)high << 8 | low;
//Serial.print(" int-val: ");
//Serial.print(value);
//     value &= ~0x0003;

//Serial.print("  wire-END;");
//Serial.println(value);
     return value;
}

float SDP6xxClass::readT()//new: remove Class

{
     return -46.85+175.72/65536.00*(float)readMeasurement(TEMP);
}
float SDP6xxClass::readPA()//new: remove Class

{
//Serial.print(" readPA: "), Serial.print(readMeasurement(PRESSURE));
//Serial.print(" readMeasurement(Pressure/1200): ")
// Serial.println(1/1200*(float)readMeasurement(PRESSURE));

	return readMeasurement(PRESSURE);
//     return 1/1200*(float)readMeasurement(PRESSURE);
}
float SDP6xxClass::readRH() //new: remove Class

{
return 1/1200*(float)readMeasurement(HUMIDITY);
// RH:     return -6.0+125.0/65536.00*(float)readMeasurement(HUMIDITY);
}



void SDP6xxClass::setHeater(uint8_t on)//new: remove Class

{
     uint8_t userRegister;
     userRegister=readUserRegister();
     if (on) {
          userRegister=(userRegister&~SDP6xx_HEATER_MASK) | SDP6xx_HEATER_ON;
     } else {
          userRegister=(userRegister&~SDP6xx_HEATER_MASK) | SDP6xx_HEATER_OFF;
     }
}
 SDP6xxClass SDP6xx;
