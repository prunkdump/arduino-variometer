#ifndef VARIOSCREEN_H
#define VARIOSCREEN_H

#include <Arduino.h>
#include <SPI.h>
#include <digit.h>

/* minimum drift to update digit */
#define VARIOSCREEN_DIGIT_DISPLAY_THRESHOLD 0.65


/********************/
/* The screen class */
/********************/

class VarioScreen {
 public:
  VarioScreen(int8_t DC, int8_t CS, int8_t RST);
  void begin(uint8_t clockDiviser = SPI_CLOCK_DIV2, uint8_t contrast = 40, uint8_t bias = 0x04);
  void beginDisplay(uint8_t x, uint8_t y);
  void display(uint8_t displayByte);
  void endDisplay();
  void flush();
  void clear();
  void beginClear(); //multi step clear
  bool clearStep(); //return true while clearing

 private:
  void command(uint8_t c);
  uint8_t clearingStep;
  int8_t _dc, _rst, _cs;
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
  void displayCharacter(const uint8_t* pointer, uint8_t width, uint8_t line);
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




/************************/
/* The screen scheduler */
/************************/
class ScreenScheduler {

 public:
 ScreenScheduler(VarioScreen& screen, VarioScreenObject** object, uint8_t* page, uint8_t objectCount)
   : screen(screen), object(object), page(page), objectCount(objectCount), pos(0), currentPage(0) { }
  void displayStep(void);
  void setPage(uint8_t page);  
  
 private:
  VarioScreen& screen;
  VarioScreenObject** object;
  uint8_t* page;
  uint8_t objectCount;
  uint8_t pos;
  uint8_t currentPage;

};


#endif
