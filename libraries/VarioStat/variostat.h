#ifndef VARIOSTAT_H
#define VARIOSTAT_H

#include <Arduino.h>

#define FLY_STAT_HEADER_EEPROM_ADDRESS 0x230 
#define FLY_STAT_EEPROM_TAG 2568

/********************/
/* The stat class   */
/********************/

class VarioStat {
 public:
   void Begin(void);
   void Enable(void);
   void Disable(void);
   void SetAlti(double alti);
   void SetSpeed(double speed);
   void SetVario(double vario);
   void SetDuration(int8_t* duree);
   double GetAlti(void);
   double GetVarioMin(void);
   double GetVarioMax(void);
   double GetSpeed(void);
   int8_t* GetDuration(void);
   void Display(void);
   bool  Handle(void);

 private:
   unsigned long Timer;
   double        MaxAlti;
   double        MaxSpeed;
   double        MaxVario;
   double        MinVario;
   bool          EnableRecord;
   int8_t 		 time[3];
     
   void ReadEeprom(void);
   void WriteEeprom(void);
};

/*int EEPROMAnythingWrite(int pos, char *character, int length);
int EEPROMAnythingRead(int pos, char *character, int length);*/

#endif
