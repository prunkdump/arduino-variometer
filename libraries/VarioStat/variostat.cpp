#include <variostat.h>
#include <Arduino.h>
#include <EEPROM.h>
#include <ToneAC.h>


// Write any data structure or variable to EEPROM
static int EEPROMAnythingWrite(int pos, char *character, int length)
{
  for (int i = 0; i < length; i++)
  {
	if (EEPROM.read(pos + i) != *character)  {
      EEPROM.write(pos + i, *character);
	}
    character++;
  }
  return pos + length;
}
 
// Read any data structure or variable from EEPROM
static int EEPROMAnythingRead(int pos, char *character, int length)
{
  for (int i = 0; i < length; i++)
  {
    *character = EEPROM.read(pos + i);
    character++;
  }
  return pos + length;
}

void VarioStat::Begin(void) {
  Timer = millis();
  MaxAlti  = 0;
  MaxSpeed = 0;
  MaxVario = 0;
  MinVario = 0;
  Enable();
}

void VarioStat::Enable() {
  EnableRecord = true;
}

void VarioStat::Disable() {
  EnableRecord = false;
}

void VarioStat::SetAlti(double alti) {
  if (alti > MaxAlti) MaxAlti = alti;
}

void VarioStat::SetSpeed(double speed) {
  if (speed > MaxSpeed) MaxSpeed = speed;
}

void VarioStat::SetVario(double vario) {
  if (vario > MaxVario) MaxVario = vario;
  if (vario < MinVario) MinVario = vario;
}

void VarioStat::SetDuration(int8_t* duree) {
  time[0] = duree[0];	
  time[1] = duree[1];	
  time[2] = duree[2];	
}
   
double VarioStat::GetAlti() {
  return MaxAlti;
}

double VarioStat::GetVarioMin() {
  return MinVario;
}

double VarioStat::GetVarioMax() {
  return MaxVario;
}

double VarioStat::GetSpeed() {
  return MaxSpeed;
}
   
int8_t* VarioStat::GetDuration() {
  return time;
}
  
void VarioStat::Display() {
//  Serial.println("Display");	
  ReadEeprom();
}

bool VarioStat::Handle() {
  if ((EnableRecord) && (millis()-Timer) > 10000) {
	  Timer = millis();
	  WriteEeprom();
	  return true;
  }
  return false;
}

void VarioStat::ReadEeprom() {
  int eepromAddress = FLY_STAT_HEADER_EEPROM_ADDRESS;
  unsigned int val_int = 0;
   
  // Integer read from EEPROM
  eepromAddress = EEPROMAnythingRead(eepromAddress, reinterpret_cast<char*>(&val_int), sizeof(val_int));
//  Serial.print("val_int : ");	
//  Serial.println(val_int); 
 
  if (val_int == FLY_STAT_EEPROM_TAG) {  
    // Float read to EEPROM
    float   val_float = 0;
    eepromAddress = EEPROMAnythingRead(eepromAddress, reinterpret_cast<char*>(&val_float), sizeof(val_float));
	MaxAlti = val_float;

    eepromAddress = EEPROMAnythingRead(eepromAddress, reinterpret_cast<char*>(&val_float), sizeof(val_float));
	MaxSpeed = val_float;

    eepromAddress = EEPROMAnythingRead(eepromAddress, reinterpret_cast<char*>(&val_float), sizeof(val_float));
	MinVario = val_float;

    eepromAddress = EEPROMAnythingRead(eepromAddress, reinterpret_cast<char*>(&val_float), sizeof(val_float));
	MaxVario = val_float;
	
    // Read array from EEPROM
    EEPROMAnythingRead(eepromAddress, reinterpret_cast<char*>(&time), sizeof(time));   
  }
  else {
    MaxAlti    = 1200;
    MaxSpeed   = 52;
    MaxVario   = 3.2;
    MinVario   = -1.8;
    time[0]    = 10;
    time[1]    = 10;
    time[2]    = 10;	  
  }
}

void VarioStat::WriteEeprom() {
	
	  /******************/
  /* save to EEPROM */
  /******************/
  int eepromAddress = FLY_STAT_HEADER_EEPROM_ADDRESS;

  // int to EEPROM
  unsigned int val_int = FLY_STAT_EEPROM_TAG;
  eepromAddress = EEPROMAnythingWrite(eepromAddress, reinterpret_cast<char*>(&val_int), sizeof(val_int));

  // Float to EEPROM
  float   val_float = MaxAlti;
  eepromAddress = EEPROMAnythingWrite(eepromAddress, reinterpret_cast<char*>(&val_float), sizeof(val_float)); 
  
  val_float = MaxSpeed;
  eepromAddress = EEPROMAnythingWrite(eepromAddress, reinterpret_cast<char*>(&val_float), sizeof(val_float)); 

  val_float = MinVario;
  eepromAddress = EEPROMAnythingWrite(eepromAddress, reinterpret_cast<char*>(&val_float), sizeof(val_float)); 

  val_float = MaxVario;
  eepromAddress = EEPROMAnythingWrite(eepromAddress, reinterpret_cast<char*>(&val_float), sizeof(val_float)); 

  eepromAddress = EEPROMAnythingWrite(eepromAddress, reinterpret_cast<char*>(&time), sizeof(time));   
  
/*  toneAC(900);
  delay(1000);
  toneAC(0);*/

}

/**
 * Read a single 16 bits integer
 *
uint16_t EEPROMClassEx::readInt(int address)
{
	if (!isReadOk(address+sizeof(uint16_t))) return 0;
	return eeprom_read_word((uint16_t *) address);
}


/*
 void EEwriteFloat(int addr, float f) {
    unsigned char *buf = (unsigned char*)(&f);
    for ( int i = 0 ; i < sizeof(f) ; i++ ) {
        EEPROM.write(addr+i, buf[i]);
    }
}
 
float EEreadFloat(int addr) {
    float f;
    unsigned char *buf = (unsigned char*)(&f);
    for ( int i = 0 ; i < sizeof(f) ; i++ ) {
         buf[i] = EEPROM.read(addr+i);
    }
    return f;
}     
*/    
  
  
