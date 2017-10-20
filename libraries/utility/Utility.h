#ifndef UTILITY_H
#define UTILITY_H

#include <Arduino.h>
#include <beeper.h>
#include <EnergySaving.h>


/******************************************************/
/******************************************************/


/*class Statistic {

 public:
   void setTime(int8_t* timeValue);
   int8_t* getTime(void);
   int8_t* getTime(int8_t* timeValue);
   void setDuration(int8_t* durationValue);
   int8_t* getDuration(void);
   int8_t* getDuration(int8_t* durationValue);
   void setAlti(double alti);
   double getMaxAlti(void);
   double getMinAlti(void);
   void setVario(double vario);
   double getMaxVario(void);
   double getMinVario(void);
   void setSpeed(double speed);
   double getMaxSpeed(void);
   double getMinSpeed(void);
   double getAltiDeco(void);
   double getGain(void);

  private:
    int8_t time[3];
	int8_t duration[3];

    double currentSpeed=0;
    double maxSpeed;
    double minSpeed;
    double currentAlti=0;
    double maxAlti;
    double minAlti;
	double currentVario=0;
	double maxVario;
	double minVario;
	
	double altiDeco;
	double gain;
};*/

extern EnergySaving nrgSave;
extern byte statePowerInt;
extern byte statePower;


/*Battery voltage * 10 */
int batteryVoltage(void);
void indicateUncalibratedAccelerometer();
void indicatePowerDown();
void indicateFaultMS5611();
void indicateFaultMPU9250();
void indicateFaultSDCARD();
void signalBeep(double freq, unsigned long duration, int count);
void powerDown();
int8_t percentBat(double targetVoltage);

class Compteur {

 public:
	void initTime();
	void updateTime();
	bool IsDelay(int delayCompteur);

 private:
	uint32_t timePreviousUs;
	uint32_t timeNowUs;
	float imuTimeDeltaSecs;
};

#endif
