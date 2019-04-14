/*********************************************************************************/
/*                                                                               */
/*                           Libraries ToneHal                                   */
/*                                                                               */
/*  version    Date     Description                                              */
/*    1.0    20/01/19                                                            */
/*    1.1    24/01/19   Réecriture des classes                                   */
/*                      répartition en plusieurs fichiers                        */
/*    1.2    26/01/19   Modifications mineures                                   */
/*    1.3    01/04/19   Ajout ESP32                                              */
/*                                                                               */
/*********************************************************************************/

#include "eepromHAL.h"
#include <Arduino.h>

#if defined (EEPROMHAL_EXTENDED)	

int EepromHal::readInt(int address){
	int value = 0x0000;
	value = value | (read(address) << 8);
	value = value | read(address+1);
   return value;
}

void EepromHal::writeInt(int address, int value){
   write(address, (value >> 8) & 0xFF );
   write(address+1, value & 0xFF);
}
 
float EepromHal::readFloat(int address){
   union u_tag {
     byte b[4];
     float fval;
   } u;   
   u.b[0] = read(address);
   u.b[1] = read(address+1);
   u.b[2] = read(address+2);
   u.b[3] = read(address+3);
   return u.fval;
}
 
void EepromHal::writeFloat(int address, float value){
   union u_tag {
     byte b[4];
     float fval;
   } u;
   u.fval=value;
 
   write(address  , u.b[0]);
   write(address+1, u.b[1]);
   write(address+2, u.b[2]);
   write(address+3, u.b[3]);
}

void EepromHal::readString(int offset, int bytes, char *buf){
  char c = 0;
  for (int i = offset; i < (offset + bytes); i++) {
    c = read(i);
    buf[i - offset] = c;
    if (c == 0) break;
  }
}

void EepromHal::writeString(int offset, int bytes, char *buf){
  char c = 0;
  //int len = (strlen(buf) < bytes) ? strlen(buf) : bytes;
  for (int i = 0; i < bytes; i++) {
    c = buf[i];
    write(offset + i, c); 
  }
}

#endif // EEPROMHAL_EXTENDED

