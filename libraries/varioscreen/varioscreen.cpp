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

#include <varioscreen.h>
#include <Arduino.h>
#include <avr/pgmspace.h>
#include <SPI.h>
#include <digit.h>

#define VARIOSCREEN_FONT_HEIGHT 2

/****************/
/* digits fonts */
/****************/
#define VARIOSCREEN_DIGIT_WIDTH 11

const uint8_t varioscreenDigit[][VARIOSCREEN_DIGIT_WIDTH*VARIOSCREEN_FONT_HEIGHT] PROGMEM = {
  {0x00, 0xe0, 0xf8, 0x18, 0x0c, 0x0c, 0x0c, 0x18, 0xf8, 0xe0, 0x00, 0x00,
   0x07, 0x1f, 0x18, 0x30, 0x30, 0x30, 0x18, 0x1f, 0x07, 0x00 },
  {0x00, 0x00, 0x18, 0x08, 0x0c, 0xfc, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00,
   0x00, 0x30, 0x30, 0x30, 0x3f, 0x3f, 0x30, 0x30, 0x30, 0x00 },
  {0x00, 0x18, 0x0c, 0x0c, 0x0c, 0x0c, 0x9c, 0xf8, 0xf0, 0x00, 0x00, 0x00,
   0x30, 0x38, 0x3c, 0x3e, 0x37, 0x33, 0x31, 0x30, 0x00, 0x00 },
  {0x00, 0x18, 0x0c, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xf8, 0x38, 0x00, 0x00,
   0x18, 0x30, 0x30, 0x30, 0x30, 0x30, 0x39, 0x1f, 0x0f, 0x00 },
  {0x00, 0x80, 0xe0, 0x70, 0x3c, 0x0c, 0xfc, 0xfc, 0x00, 0x00, 0x00, 0x00,
   0x0f, 0x0d, 0x0c, 0x0c, 0x0c, 0x3f, 0x3f, 0x0c, 0x0c, 0x00 },
  {0x00, 0xfc, 0xfc, 0xcc, 0xcc, 0xcc, 0xcc, 0x8c, 0x8c, 0x00, 0x00, 0x00,
   0x19, 0x30, 0x30, 0x30, 0x30, 0x30, 0x39, 0x1f, 0x0f, 0x00 },
  {0x00, 0xe0, 0xf0, 0x98, 0xcc, 0xcc, 0xcc, 0xcc, 0x98, 0x00, 0x00, 0x00,
   0x07, 0x1f, 0x19, 0x30, 0x30, 0x30, 0x39, 0x1f, 0x0f, 0x00 },
  {0x00, 0x0c, 0x0c, 0x0c, 0x0c, 0x0c, 0x8c, 0xfc, 0x7c, 0x1c, 0x00, 0x00,
   0x00, 0x00, 0x20, 0x38, 0x3e, 0x07, 0x01, 0x00, 0x00, 0x00 },
  {0x00, 0x38, 0xf8, 0xcc, 0xcc, 0xcc, 0xcc, 0xcc, 0xf8, 0x38, 0x00, 0x00,
   0x0f, 0x1f, 0x39, 0x30, 0x30, 0x30, 0x39, 0x1f, 0x0f, 0x00 },
  {0x00, 0xf0, 0xf8, 0x9c, 0x0c, 0x0c, 0x0c, 0x98, 0xf8, 0xe0, 0x00, 0x00,
   0x00, 0x19, 0x33, 0x33, 0x33, 0x33, 0x19, 0x0f, 0x07, 0x00 }
};

const uint8_t varioscreenPlus[] PROGMEM = {
  0x00, 0x00, 0x80, 0x80, 0x80, 0xf0, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };

const uint8_t varioscreenMinus[] PROGMEM = {
  0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

/*********************/
/* units and symbols */
/*********************/

#define VARIOSCREEN_DOT_WIDTH 6
const uint8_t varioscreenDot[] PROGMEM = {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x38, 0x38, 0x00, 0x00 };

#define VARIOSCREEN_MS_WIDTH 24
const uint8_t varioscreenMS[] PROGMEM = {
  0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00,
  0x00, 0x00, 0xc0, 0x60, 0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00,
  0x00, 0x3f, 0x3f, 0x00, 0x00, 0x3f, 0x3f, 0x00, 0x00, 0x3f, 0x3f, 0x00,
  0x60, 0x3c, 0x03, 0x00, 0x00, 0x13, 0x27, 0x24, 0x24, 0x3c, 0x19, 0x00 };

#define VARIOSCREEN_M_WIDTH 12
const uint8_t varioscreenM[] PROGMEM = {
  0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00, 0x00,
  0x00, 0x3f, 0x3f, 0x00, 0x00, 0x3f, 0x3f, 0x00, 0x00, 0x3f, 0x3f, 0x00 };

#define VARIOSCREEN_KMH_WIDTH 11
const uint8_t varioscreenKMH[] PROGMEM = {
  0x00, 0x7c, 0x20, 0x50, 0x00, 0x70, 0x10, 0x60, 0x10, 0x60, 0x00, 0x00,
  0x01, 0x01, 0x01, 0x7d, 0x11, 0x11, 0x61, 0x01, 0x01, 0x00 };

#define VARIOSCREEN_GR_WIDTH 12
const uint8_t varioscreenGR[] PROGMEM = {
  0x00, 0xc0, 0xe0, 0x30, 0x90, 0x90, 0xb0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x01, 0x03, 0x06, 0x04, 0x07, 0x07, 0x00, 0x78, 0x08, 0x08, 0x00 };

#define VARIOSCREEN_MUTE_WIDTH 14 
const uint8_t varioscreenMute[] PROGMEM = {
  0x00, 0x0c, 0x18, 0xf0, 0x60, 0xe0, 0xb0, 0x18, 0x0c, 0xfe, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0x04, 0x07, 0x0d, 0x1b, 0x36, 0x7f,
  0x18, 0x30, 0x60, 0x00 };


/**********/
/* screen */
/**********/
#define LCDWIDTH 84
#define LCDHEIGHT 6
#define PCD8544_POWERDOWN 0x04
#define PCD8544_ENTRYMODE 0x02
#define PCD8544_EXTENDEDINSTRUCTION 0x01
#define PCD8544_DISPLAYBLANK 0x0
#define PCD8544_DISPLAYNORMAL 0x4
#define PCD8544_DISPLAYALLON 0x1
#define PCD8544_DISPLAYINVERTED 0x5
// H = 0
#define PCD8544_FUNCTIONSET 0x20
#define PCD8544_DISPLAYCONTROL 0x08
#define PCD8544_SETYADDR 0x40
#define PCD8544_SETXADDR 0x80
// H = 1
#define PCD8544_SETTEMP 0x04
#define PCD8544_SETBIAS 0x10
#define PCD8544_SETVOP 0x80

void VarioScreen::enableSPI(void) {

  /* prevent enabling hardware SS pin */ 
  pinMode(SS, OUTPUT);

  /* set real CS line */
  digitalWrite(_cs, HIGH);
  pinMode(_cs, OUTPUT);

  /* set SPI */ 
  SPI.begin();
}

void VarioScreen::startSPI(void) {
  if( ! spiStarted ) {
    SPI.beginTransaction(VARIOSCREEN_SPI_SETTINGS);
    digitalWrite(_cs, LOW);
    spiStarted = true;
  }
}

void VarioScreen::stopSPI(void) {
  if( spiStarted ) {
    digitalWrite(_cs, HIGH);
    SPI.endTransaction();
    spiStarted = false;
  }
}
  

void VarioScreen::begin(uint8_t contrast, uint8_t bias) {

  // ONLY HARDWARE SPI 
  enableSPI();

  // Set common pin outputs.
  pinMode(_dc, OUTPUT);
  pinMode(_rst, OUTPUT);

  // toggle RST low to reset
  digitalWrite(_rst, LOW);
  delay(500);
  digitalWrite(_rst, HIGH);
  
  // get into the EXTENDED mode!
  command(PCD8544_FUNCTIONSET | PCD8544_EXTENDEDINSTRUCTION );

  // LCD bias select (4 is optimal?)
  command(PCD8544_SETBIAS | bias);

  // set VOP
  if (contrast > 0x7f)
    contrast = 0x7f;

  command( PCD8544_SETVOP | contrast); // Experimentally determined

  // normal mode
  command(PCD8544_FUNCTIONSET);

  // Set display to Normal
  command(PCD8544_DISPLAYCONTROL | PCD8544_DISPLAYNORMAL);

  // reset display
  clear();
  stopSPI();
}

/********************/
/* display sequence */
/********************/
void VarioScreen::beginDisplay(uint8_t x, uint8_t y) {
  command(PCD8544_SETYADDR | y);
  command(PCD8544_SETXADDR | x);
  digitalWrite(_dc, HIGH);
}

void VarioScreen::display(uint8_t displayByte) {
  SPI.transfer(displayByte);
}

//do nothing with the new SPI library
void VarioScreen::endDisplay() {
  
}

void VarioScreen::flush() {
  command(PCD8544_SETYADDR);
  stopSPI();
}

/*********/
/* clear */
/*********/
void VarioScreen::clear() {
  beginDisplay(0, 0);
  for(int i = 0; i<LCDWIDTH*LCDHEIGHT; i++) {
    display(0x00);
  }
  endDisplay();
  flush();
}

void VarioScreen::beginClear() {
  clearingStep = 0;
}

bool VarioScreen::clearStep() {

  /* check if clear is needed */
  if( clearingStep == LCDHEIGHT ) {
    return false;
  }

  /* clear one line */
  beginDisplay(0, clearingStep);
  for(int i = 0; i<LCDWIDTH; i++) {
    display(0x00);
  }
  endDisplay();
  flush();

  /* next */
  clearingStep++;
  return true;
}
  
void VarioScreen::command(uint8_t c) {
  startSPI();
  digitalWrite(_dc, LOW);
  SPI.transfer(c);
}


/*****************************************/
/*  methods common to all screen objects */
/*****************************************/
#define display_needed() (state & 0x01)
#define display_done() state &= ~(0x01)
#define setDisplayFlag() state |= (0x01)

bool VarioScreenObject::update(void) {

  if( display_needed() ) {
    display();
    display_done();
    return true;
  }

  return false;
}

void VarioScreenObject::reset(void) {
  setDisplayFlag();
}


/*******************/
/*  screen objects */
/*******************/

/* digit */
#define MAX_CHAR_IN_LINE 7

void ScreenDigit::setValue(double value) {

  /* build digit and check changes */
  if( digit.begin(value) ) {
    reset();
  }
}
  

void ScreenDigit::display() {

  /***************/
  /* build digit */
  /***************/

  /* check if digit need to be rebuilded */
  if( ! digit.available() ) {
    digit.rebuild();
  }

  /* to store the characters to be displayed */
  uint8_t digitCharacters[MAX_CHAR_IN_LINE];
  uint8_t digitSize = 0;

  /* compute the total digit width in pixels */
  uint8_t digitWidth = digit.size(VARIOSCREEN_DIGIT_WIDTH, VARIOSCREEN_DIGIT_WIDTH, VARIOSCREEN_DOT_WIDTH);

  /* get characters */
  while( digit.available() ) {
    digitCharacters[digitSize] = digit.get();
    digitSize++;
  }
   
  /*******************/
  /* prepare display */
  /*******************/

  /* number of bytes that need to be deleted */
  uint8_t nullWidth;
  if( digitWidth < lastDisplayWidth ) {
    nullWidth = lastDisplayWidth - digitWidth;
  } else {
    nullWidth = 0;
  }
  lastDisplayWidth = digitWidth;

  /* start position */
  uint8_t posX = anchorX - digitWidth - nullWidth;
  uint8_t posY = anchorY;
  
  /*****************/
  /* start display */
  /*****************/
  
  uint8_t line;
  for(line = 0; line < VARIOSCREEN_FONT_HEIGHT; line++) {
    screen.beginDisplay(posX, posY + line);

    /* the null bytes */
    for(int i =0; i<nullWidth; i++) {
      screen.display(0);
    }

    /* the digit characters */
    for(int i = 0; i<digitSize; i++) {
      uint8_t c = digitCharacters[i];
      
      switch( c ) {
      case '+':
	displayElementLine(varioscreenPlus, VARIOSCREEN_DIGIT_WIDTH, line);
	break;

      case '-':
	displayElementLine(varioscreenMinus, VARIOSCREEN_DIGIT_WIDTH, line);
	break;

      case '.':
	displayElementLine(varioscreenDot, VARIOSCREEN_DOT_WIDTH, line);
	break;

      default:
	displayElementLine(varioscreenDigit[c - '0'], VARIOSCREEN_DIGIT_WIDTH, line);
	break;
      }
    }
    
    screen.endDisplay();
  }
  
  screen.flush();
}


void VarioScreenObject::displayElementLine(const uint8_t* pointer, uint8_t width, uint8_t line) {
  for(int i = 0; i<width; i++) {
    screen.display( pgm_read_byte( &pointer[i + line * width] ) );
  }
}

void VarioScreenObject::displayElement(VarioScreen& screen, const uint8_t* pointer, uint8_t posX, uint8_t posY, uint8_t width, uint8_t height) {
  for(int line=0; line<height; line++) {
    screen.beginDisplay(posX, posY + line);
    displayElementLine(pointer, width, line);
    screen.endDisplay();
  }
  screen.flush();
}


void MSUnit::display() {
  
  displayElement(screen, varioscreenMS, posX, posY, VARIOSCREEN_MS_WIDTH, VARIOSCREEN_FONT_HEIGHT);
}

void MUnit::display() {
  
  displayElement(screen, varioscreenM, posX, posY, VARIOSCREEN_M_WIDTH, VARIOSCREEN_FONT_HEIGHT);
}

void KMHUnit::display() {
  
  displayElement(screen, varioscreenKMH, posX, posY, VARIOSCREEN_KMH_WIDTH, VARIOSCREEN_FONT_HEIGHT);
}


void GRUnit::display() {
 
  displayElement(screen, varioscreenGR, posX, posY, VARIOSCREEN_GR_WIDTH, VARIOSCREEN_FONT_HEIGHT);
}


/* !!! always reset !!! */
void BATLevel::setVoltage(int voltage) {

  /* shift voltage to 16 bit */
  uVoltage = (uint16_t)voltage;
  uVoltage <<= VARIOSCREEN_BAT_MULTIPLIER;
  
  reset();
}

void BATLevel::display(void) {

  /* battery base */
  screen.beginDisplay(posX, posY);
  screen.display(0xff);

  /* battery level */
  uint16_t baseVoltage = base + inc;
  uint8_t pixelCount = 0;

  while( pixelCount < VARIOSCREEN_BAT_PIXEL_COUNT ) {
    if( baseVoltage < uVoltage ) {
      screen.display( 0xff );
    } else {
      screen.display( 0x81 );
    }

    baseVoltage += inc;
    pixelCount++;
  }

  /* battery end */
  screen.display( 0xff  );
  screen.display( 0x3c  );
  screen.display( 0x3c );
  screen.endDisplay();
  
  screen.flush();
}



/* !!! always reset !!! */
void SATLevel::setSatelliteCount(uint8_t count) {

  satelliteCount = count;
  reset();
}
  
void SATLevel::display(void) {

  screen.beginDisplay(posX, posY);

  uint8_t satCmp = 1;
  uint8_t satBar = 0x60;
  while( satCmp < 16 ) {

    /* check level and display bar or blank */
    if( satelliteCount > satCmp ) {
      screen.display(satBar);
      screen.display(satBar);
    } else {
      screen.display(0x00);
      screen.display(0x00);
    }

    /* separator */
    screen.display(0x00);

    /* next comparison */
    satCmp <<= 1;
    satBar = (satBar >> 2) + 0x60;
  }
   
  screen.endDisplay();
  screen.flush();
}

/* time */
void ScreenTime::setTime(uint32_t newTime) {

  for( uint8_t i = 0; i<3; i++) {
    time[i] = (int8_t)(newTime % 100);
    newTime /= 100;
  }
}

void ScreenTime::setTime(int8_t* newTime) {

  for(uint8_t i = 0; i<3; i++) {
    time[i] = newTime[i];
  }
}

void ScreenTime::correctTimeZone(int8_t UTCDrift) {
  
  time[2] += UTCDrift;
  if( time[2] < 0 ) {
    time[2] += 24;
  }
  if( time[2] >= 24 ) {
    time[2] -= 24;
  }
}

int8_t* ScreenTime::getTime(void) {

  return time;
}

/* !!! never reset, only on page change !!! */
void ScreenTime::display(void) {

  /* for each line */
  for(uint8_t line = 0; line<VARIOSCREEN_FONT_HEIGHT; line++ ) {
    screen.beginDisplay(posX, posY + line);

    /* display each time element */
    for(int8_t i = 2; i>=0; i--) {

      /* the digit */
      int8_t c;
      c = time[i]/10;
      displayElementLine(varioscreenDigit[c], VARIOSCREEN_DIGIT_WIDTH, line);
      c = time[i]%10;
      displayElementLine(varioscreenDigit[c], VARIOSCREEN_DIGIT_WIDTH, line);

      /* the dot (not at the end) */
      if( i ) {
	screen.display(0x18);
	screen.display(0x18);
      }
    }
    screen.endDisplay();
  }
  screen.flush();
}


void ScreenElapsedTime::setBaseTime(int8_t* time) {

  for(uint8_t i = 0; i<3; i++) {
    baseTime[i] = time[i];
  }
}

void ScreenElapsedTime::setCurrentTime(int8_t* currentTime) {

  /* compute elapsed time */
  int8_t rem = 0;
  int8_t v;
  for(uint8_t i = 0; i<3; i++) {
    v = (currentTime[i] - baseTime[i]) - rem;
    if( v < 0 ) {
      v += 60;
      rem = 1;
    } else {
      rem = 0;
    }
    time[i] = v;
  }
}

void ScreenMuteIndicator::setMuteState(bool newState) {

  /* set new state and reset */
  muted = newState;
  reset();
}

void ScreenMuteIndicator::display(void) {

  /* if muted display symbol */
  if( muted ) {
    displayElement(screen, varioscreenMute, posX, posY, VARIOSCREEN_MUTE_WIDTH, VARIOSCREEN_FONT_HEIGHT);
  }

  /* else erase the symbol */
  else {
    for(int line = 0; line < VARIOSCREEN_FONT_HEIGHT; line++) { 
      screen.beginDisplay(posX, posY + line);
      for(int i = 0; i<VARIOSCREEN_MUTE_WIDTH; i++) {
	screen.display(0x00);
      }
      screen.endDisplay();
    }
  }

  /* done */
  screen.flush();
}

/************************/
/* The screen scheduler */
/************************/

void ScreenScheduler::displayStep(void) {

  /* first check if screen need to be cleared */
  if( screen.clearStep() ) {
    return;
  }

  /* next try to find something to display */
  /* for the current page                  */
  uint8_t n = 0;
  while( n != objectCount ) {
    if( displayList[pos].page == currentPage && displayList[pos].object->update() ) {
      return;
    }

    /* next */
    pos++;
    if( pos == objectCount) {
      pos = 0;
    }
    n++;
  }
}

int8_t ScreenScheduler::getPage(void) {

  return currentPage;
}

void ScreenScheduler::setPage(int8_t page)  {

  /* check if page change is needed */
  if( page == currentPage ) {
    return;
  }

  /* set the new page */
  currentPage = page;

  /* screen need to by cleared */
  screen.beginClear();

  /* all the page object need to be redisplayed */
  /* but no problem to reset all the objects */
  for(uint8_t i = 0; i<objectCount; i++) {
    displayList[i].object->reset();
  }
}

void ScreenScheduler::nextPage(void) {

  uint8_t newPage = currentPage + 1;
  if( newPage > endPage ) {
    newPage = 0;
  }

  setPage(newPage);
}
    
      
    
  
  
