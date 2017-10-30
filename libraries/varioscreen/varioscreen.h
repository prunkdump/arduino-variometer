#ifndef VARIOSCREEN_H
#define VARIOSCREEN_H

#include <Arduino.h>
#include <digit.h>
// select the display class to use, only one
#include <GxGDEP015OC1/GxGDEP015OC1NL.h>
//#include <GxGDE0213B1/GxGDE0213B1.h>
//#include <GxGDEH029A1/GxGDEH029A1.h>
//#include <GxGDEW042T2/GxGDEW042T2.h>

// FreeFonts from Adafruit_GFX
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>

/* minimum drift to update digit */
#define VARIOSCREEN_DIGIT_DISPLAY_THRESHOLD 0.65

/*  lipo battery percentage to voltage linear interpolation */
/*    voltage = a * percent + b                             */ 
#define VARIOSCREEN_BAT_A_COEFF 0.0059
#define VARIOSCREEN_BAT_B_COEFF 3.5534
#define VARIOSCREEN_BAT_PIXEL_COUNT 10
#define VARIOSCREEN_BAT_MULTIPLIER 6

/********************/
/* The screen class */
/********************/

class VarioScreen : public GxEPD_Class {
 public:
  VarioScreen(GxIO& io, uint8_t rst, uint8_t busy) 
    : GxEPD_Class(io, rst, busy) {}
  
  void begin();
  void  getTextBounds(char *string, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w, uint16_t *h);
  void updateScreen (void);
//  void clearScreen(void); 

 /* void beginClear(void); //multi step clear
  bool clearStep(void); //return true while clearing*/
  
// private:
//  uint8_t clearingStep;
};


/* the main abstract class for all the objects */ 
/*----------------------------*/
class VarioScreenObject {

 public:
  bool update(void);  //return true if an update was done
  void reset(void);   //force redisplay 
  virtual void display(void) = 0;
  
 protected:
  VarioScreenObject(VarioScreen& screen, uint8_t state) : screen(screen), state(state) { }
  VarioScreen& screen;
  uint8_t state; //the first byte is used to know is redisplay is needed
                 //the other can be used freely
};

/* screen digit */

#define ALIGNNONE  0
#define ALIGNSPACE 1
#define ALIGNZERO  2

class ScreenDigit: public VarioScreenObject {

 public :
 ScreenDigit(VarioScreen& screen, uint16_t anchorX, uint16_t anchorY, uint16_t width, uint16_t precision, boolean plusDisplay = false, boolean zero = false, boolean leftAlign = false)
   : VarioScreenObject(screen, 0), anchorX(anchorX), anchorY(anchorY), width(width), precision(precision), plusDisplay(plusDisplay), zero(zero), leftAlign(leftAlign)
  { lastDisplayWidth = 0; }
  void display(void);
  void setValue(double value);
   
 private:
  int digitsBe4Decimal(double number);
  char * dtostrf2(double number, signed char width, unsigned char prec, char *s, boolean zero);
  double value;
  double oldvalue=0;
  const uint16_t anchorX, anchorY, precision, width;
  boolean plusDisplay, zero, leftAlign;
  uint8_t lastDisplayWidth;
};

/* meters unit */
class MUnit : public VarioScreenObject {

 public :
  MUnit(VarioScreen& screen, uint8_t posX, uint8_t posY)
   : VarioScreenObject(screen, 1), posX(posX), posY(posY) { }
  void display(void);
  
 private :
  const uint8_t posX;
  const uint8_t posY;
};

/* meters per second unit */
class MSUnit : public VarioScreenObject {

 public :
 MSUnit(VarioScreen& screen, uint8_t posX, uint8_t posY)
   : VarioScreenObject(screen, 1), posX(posX), posY(posY) { }
  void display(void);
  
 private :
  const uint8_t posX;
  const uint8_t posY;
};

/* kilometers per hour unit */
class KMHUnit : public VarioScreenObject {

 public :
  KMHUnit(VarioScreen& screen, uint8_t posX, uint8_t posY)
   : VarioScreenObject(screen, 1), posX(posX), posY(posY) { }
  void display(void);

 private :
  const uint8_t posX;
  const uint8_t posY;
};

/* battery level */
class BATLevel : public VarioScreenObject {

 public :
 BATLevel(VarioScreen& screen, uint8_t posX, uint8_t posY, double divisor, double refVoltage)
   : VarioScreenObject(screen, 0), posX(posX), posY(posY),
    base((uint16_t)((VARIOSCREEN_BAT_B_COEFF * 1023 * (1<<VARIOSCREEN_BAT_MULTIPLIER))/(divisor*refVoltage))),
    inc((uint16_t)((VARIOSCREEN_BAT_A_COEFF * 1023 * 100 * (1<<VARIOSCREEN_BAT_MULTIPLIER))/(divisor*refVoltage*(VARIOSCREEN_BAT_PIXEL_COUNT+1))))
    { }

  void setVoltage(int voltage = 0);
  void display(void);

 private :
  uint16_t uVoltage;
  const uint8_t posX;
  const uint8_t posY;
  const uint16_t base;
  const uint16_t inc;
};

/* battery level */
class VOLLevel : public VarioScreenObject {

 public :
 VOLLevel(VarioScreen& screen, uint8_t posX, uint8_t posY)
   : VarioScreenObject(screen, 0), posX(posX), posY(posY)   { }

  void setVolume(int Volume = 0);
  void display(void);

 private :
  uint8_t volume=0;
  const uint8_t posX;
  const uint8_t posY;
};

/* satellite level */
class SATLevel : public VarioScreenObject {

 public:
 SATLevel(VarioScreen& screen, uint8_t posX, uint8_t posY)
   : VarioScreenObject(screen, 0), posX(posX), posY(posY) { }

  void setSatelliteCount(uint8_t count);
  void display(void);
  
 private:
  const uint8_t posX;
  const uint8_t posY;
  uint8_t satelliteCount;
};


/* info level */

#define INFO_NONE  0
#define INFO_USB   1


class INFOLevel : public VarioScreenObject {

 public:
 INFOLevel(VarioScreen& screen, uint8_t posX, uint8_t posY)
   : VarioScreenObject(screen, 0), posX(posX), posY(posY) { }

  void set(uint8_t value);
  void display(void);
  
 private:
  const uint8_t posX;
  const uint8_t posY;
  uint8_t InfoValue = INFO_USB;
};

/* record indicator */

#define STATE_SCAN   0
#define STATE_RECORD 1
#define STATE_GPSFIX 2


class RECORDIndicator : public VarioScreenObject {

 public:
 RECORDIndicator(VarioScreen& screen, uint8_t posX, uint8_t posY)
   : VarioScreenObject(screen, 0), posX(posX), posY(posY) { }

  void stateRECORD(void);
  void setActifSCAN(void);
  void setActifRECORD(void);
  void setActifGPSFIX(void);
  void display(void);
  
 private:
  const uint8_t posX;
  const uint8_t posY;
  bool  displayRecord=false;
  uint8_t  recordState = STATE_SCAN;
  unsigned long lastFreqUpdate=0;
};


/* trend */
class TRENDLevel : public VarioScreenObject {

 public :
  TRENDLevel(VarioScreen& screen, uint8_t posX, uint8_t posY)
   : VarioScreenObject(screen, 1), posX(posX), posY(posY) { }
  void stateTREND(int8_t state);
  void display(void);

 private :
  const uint8_t posX;
  const uint8_t posY;
  int8_t trendState = 0;
};

/* time */
class ScreenTime : public VarioScreenObject {

 public:
  ScreenTime(VarioScreen& screen, uint8_t posX, uint8_t posY, ScreenDigit& hour, ScreenDigit& minute, bool dot_or_h=false)
    : VarioScreenObject(screen, 0), posX(posX), posY(posY), hour(hour), minute(minute), dot_or_h(dot_or_h) { }

  void setTime(uint32_t newTime);
  void setTime(int8_t* newTime);
  void correctTimeZone(int8_t UTCDrift);
  int8_t* getTime(void);
  void display(void);

 protected:
  const uint8_t posX;
  const uint8_t posY;
  int8_t time[3];
  bool dot_or_h=false;
  ScreenDigit& hour, minute;
};


class ScreenElapsedTime : public ScreenTime {

 public:
  ScreenElapsedTime(VarioScreen& screen, uint8_t posX, uint8_t posY, ScreenDigit& hour, ScreenDigit& minute) 
  : ScreenTime(screen, posX, posY, hour, minute, true) { }
  
  void setBaseTime(int8_t* time);
  void setCurrentTime(int8_t* time);

 protected:
  int8_t baseTime[3];
  
};

class FIXGPSInfo : public VarioScreenObject {

 public :
  FIXGPSInfo(VarioScreen& screen, uint8_t posX, uint8_t posY)
   : VarioScreenObject(screen, 1), posX(posX), posY(posY) { }
  void setFixGps(void);
  void unsetFixGps(void);
  boolean getFixGps(void);

  void display(void);

 private :
  boolean FixGPS = false;
  const uint8_t posX;
  const uint8_t posY;
};

class BTInfo : public VarioScreenObject {

 public :
  BTInfo(VarioScreen& screen, uint8_t posX, uint8_t posY)
   : VarioScreenObject(screen, 1), posX(posX), posY(posY) { }

  void setBT(void);
  void unsetBT(void);
  boolean getBT(void);

  void display(void);

 private :
  boolean bt = false;
  const uint8_t posX;
  const uint8_t posY;
};
#endif

/************************/
/* The screen scheduler */
/************************/

/* une negative page index for pages displayed once */
struct ScreenSchedulerObject {
  
  VarioScreenObject* object;
  int8_t page;
  boolean actif;
};

/* the scheduler loop on 0 <--> endPage */
class ScreenScheduler {

 public:
 ScreenScheduler(VarioScreen& screen,
		 ScreenSchedulerObject* displayList, uint8_t objectCount, int8_t startPage, int8_t endPage)
   : screen(screen), displayList(displayList), objectCount(objectCount), pos(0), currentPage(startPage), endPage(endPage) { }
   
  void displayStep(void);
  int8_t getPage(void);
  int8_t getMaxPage(void);
  void setPage(int8_t page, boolean force = false);
  void nextPage(void);
  void previousPage(void);
  
 private:
  //const 
  VarioScreen& screen;
  const ScreenSchedulerObject* displayList;
  const uint8_t objectCount;
  uint8_t pos;
  int8_t currentPage;
  const int8_t endPage;
};

struct MultiDisplayObject {
  
  int8_t* objectList;
  const uint8_t objectCount;
  int8_t  displayActif; 
  int8_t  oldDisplayActif;
  uint8_t seconde;
  uint8_t countTime;
};

class MultiDisplay {
	
 public:

 MultiDisplay(ScreenSchedulerObject* displayList, uint8_t objectCount, MultiDisplayObject* multiDisplayList, uint8_t multiObjectCount)
   : displayList(displayList), objectCount(objectCount), multiDisplayList(multiDisplayList), multiObjectCount(multiObjectCount) { }
   
  void displayStep(void);
  
 private:
  ScreenSchedulerObject* displayList;
  const uint8_t objectCount;
  MultiDisplayObject* multiDisplayList;
  const uint8_t multiObjectCount;
  unsigned long lastFreqUpdate=0;  
};
