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

#ifndef SDP6xx_H
#define SDP6xx_H

#include <inttypes.h>
#include <Wire.h>
#include "Arduino.h"

//  Address
const uint8_t SDP6xxADDR = 0x40;

//  CRC
const uint16_t POLYNOMIAL = 0x131;  //P(x)=x^8+x^5+x^4+1 = 100110001

// sensor commands
typedef enum {
     MEASUREMENT_PA_HM   = 0xF1, // update for pressure measurement, hold master
     MEASUREMENT_T_HM    = 0xE3, // Temperature measurement, hold master
     MEASUREMENT_RH_HM   = 0xE5, // Humidity measurement,  hold master
     MEASUREMENT_T_POLL  = 0xF3, // Temperature measurement, no hold master, currently not used
     MEASUREMENT_RH_POLL = 0xF5, // Humidity measurement, no hold master, currently not used
     USER_REG_W          = 0xE6, // write user register
     USER_REG_R          = 0xE7, // read user register
     SOFT_RESET          = 0xFE  // soft reset
} SDP6xxCommand;

// sensor resolutions
typedef enum {
     SDP6xx_RES_12_14BIT       = 0x00, // RH=12bit, T=14bit
     SDP6xx_RES_8_12BIT        = 0x01, // RH= 8bit, T=12bit
     SDP6xx_RES_10_13BIT       = 0x80, // RH=10bit, T=13bit
     SDP6xx_RES_11_11BIT       = 0x81, // RH=11bit, T=11bit
     SDP6xx_RES_MASK           = 0x81  // Mask for res. bits (7,0) in user reg.
} SDP6xxResolution;

typedef enum {
     SDP6xx_HEATER_ON          = 0x04, // heater on
     SDP6xx_HEATER_OFF         = 0x00, // heater off
     SDP6xx_HEATER_MASK        = 0x04, // Mask for Heater bit(2) in user reg.
} SDP6xxHeater;

// measurement signal selection
typedef enum {
     HUMIDITY,
     TEMP,
	PRESSURE
} SDP6xxMeasureType;


class SDP6xxClass { //new: old was SDP6xxClass
public:
     void softReset();
     void setHeater(uint8_t on);
     float readRH();
     float readT();
     float readPA();

private:
     uint8_t readUserRegister();
     void writeUserRegister(uint8_t userRegister);
     uint16_t readMeasurement(SDP6xxMeasureType type);
};

 extern SDP6xxClass SDP6xx;

#endif
