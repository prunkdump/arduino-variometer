/* varioscreen -- Nokia 5110 screen library 
 *
 * Copyright 2016-2019 Baptiste PELLEGRIN
 * 
 * This file is part of GNUVario.
 *
 * GNUVario is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNUVario is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef VARIOSCREEN_H
#define VARIOSCREEN_H

#include <Arduino.h>
#include <SPI.h>
#include <digit.h>

#define VARIOSCREEN_MAX_SPI_FREQ F_CPU
#define VARIOSCREEN_SPI_SETTINGS SPISettings(F_CPU, MSBFIRST, SPI_MODE0)

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

class VarioScreen {
 public:
 VarioScreen(int8_t DC, int8_t CS, int8_t RST) : clearingStep(0), _dc(DC), _rst(RST), _cs(CS), spiStarted(false) { }
  void enableSPI(void);
  void begin(uint8_t contrast = 40, uint8_t bias = 0x04);
  void beginDisplay(uint8_t x, uint8_t y);
  void display(uint8_t displayByte);
  void endDisplay();
  void flush();
  void clear();
  void beginClear(); //multi step clear
  bool clearStep(); //return true while clearing

 private:
  void command(uint8_t c);
  void startSPI(void);
  void stopSPI(void);
  uint8_t clearingStep;
  const int8_t _dc, _rst, _cs;
  bool spiStarted;
};


/**********************/
/* The screen objects */
/**********************/

/* the main abstract class for all the objects */ 
/*----------------------------*/
class VarioScreenObject {

 public:
  bool update(void);  //return true if an update was done
  void reset(void);   //force redisplay 
  virtual void display(void) = 0;
  
 protected:
 VarioScreenObject(VarioScreen& screen, uint8_t state) : screen(screen), state(state) { }
  void displayElementLine(const uint8_t* pointer, uint8_t width, uint8_t line);
  void displayElement(VarioScreen& screen, const uint8_t* pointer, uint8_t posX, uint8_t posY, uint8_t width, uint8_t height);
  VarioScreen& screen;
  uint8_t state; //the first byte is used to know is redisplay is needed
                 //the other can be used freely
};
/*----------------------------*/


/* screen digit */
class ScreenDigit: public VarioScreenObject {

 public :
 ScreenDigit(VarioScreen& screen, uint8_t anchorX, uint8_t anchorY, uint8_t precision, boolean plusDisplay = false)
   : VarioScreenObject(screen, 0), digit(precision, plusDisplay), anchorX(anchorX), anchorY(anchorY)
  { lastDisplayWidth = 0; }
  void display(void);
  void setValue(double value);
  
 private:
  FPSDigit digit;
  const uint8_t anchorX, anchorY;
  uint8_t lastDisplayWidth;
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


/* glide ratio unit */
class GRUnit : public VarioScreenObject {

 public :
  GRUnit(VarioScreen& screen, uint8_t posX, uint8_t posY)
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

/* time */
class ScreenTime : public VarioScreenObject {

 public:
  ScreenTime(VarioScreen& screen, uint8_t posX, uint8_t posY)
    : VarioScreenObject(screen, 0), posX(posX), posY(posY) { }

  void setTime(uint32_t newTime);
  void setTime(int8_t* newTime);
  void correctTimeZone(int8_t UTCDrift);
  int8_t* getTime(void);
  void display(void);

 protected:
  const uint8_t posX;
  const uint8_t posY;
  int8_t time[3];
};


class ScreenElapsedTime : public ScreenTime {

 public:
  ScreenElapsedTime(VarioScreen& screen, uint8_t posX, uint8_t posY) 
  : ScreenTime(screen, posX, posY) { }

  void setBaseTime(int8_t* time);
  void setCurrentTime(int8_t* time);

 protected:
  int8_t baseTime[3];
  
};

class ScreenMuteIndicator : public VarioScreenObject {

 public:
 ScreenMuteIndicator(VarioScreen& screen, uint8_t posX, uint8_t posY)
   : VarioScreenObject(screen, 0), posX(posX), posY(posY), muted(false) { }

  void setMuteState(bool newState);
  void display(void);
  
 private:
  const uint8_t posX;
  const uint8_t posY;
  bool muted;
};

/************************/
/* The screen scheduler */
/************************/

/* une negative page index for pages displayed once */
struct ScreenSchedulerObject {
  
  VarioScreenObject* object;
  int8_t page;
};

/* the scheduler loop on 0 <--> endPage */
class ScreenScheduler {

 public:
 ScreenScheduler(VarioScreen& screen,
		 ScreenSchedulerObject* displayList, uint8_t objectCount, int8_t startPage, int8_t endPage)
   : screen(screen), displayList(displayList), objectCount(objectCount), pos(0), currentPage(startPage), endPage(endPage) { }
   
  void displayStep(void);
  int8_t getPage(void);
  void setPage(int8_t page);
  void nextPage(void);
  
 private:
  VarioScreen& screen;
  const ScreenSchedulerObject* displayList;
  const uint8_t objectCount;
  uint8_t pos;
  int8_t currentPage;
  const int8_t endPage;
};


#endif
