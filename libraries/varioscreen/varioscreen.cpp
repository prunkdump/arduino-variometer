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

VarioScreen::VarioScreen(int8_t DC, int8_t CS, int8_t RST) {
  _dc = DC;
  _rst = RST;
  _cs = CS;
}

void VarioScreen::begin(uint8_t clockDiviser, uint8_t contrast, uint8_t bias) {
  // ONLY HARDWARE SPI 
  SPI.begin();
  SPI.setClockDivider(clockDiviser);
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);

  // Set common pin outputs.
  pinMode(_dc, OUTPUT);
  pinMode(_rst, OUTPUT);
  pinMode(_cs, OUTPUT);

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
  this->clear();
}

void VarioScreen::beginDisplay(uint8_t x, uint8_t y) {
  command(PCD8544_SETYADDR | y);
  command(PCD8544_SETXADDR | x);
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
}

void VarioScreen::display(uint8_t displayByte) {
  SPI.transfer(displayByte);
}

void VarioScreen::endDisplay() {
  digitalWrite(_cs, HIGH);
}

void VarioScreen::flush() {
  command(PCD8544_SETYADDR);
}

void VarioScreen::clear() {
  for(int line = 0; line<LCDHEIGHT; line++) {
    this->beginDisplay(0, line);
    for(int i = 0; i<LCDWIDTH; i++) {
      this->display(0);
    }
    this->endDisplay();
  }
  this->flush();  
}

void VarioScreen::command(uint8_t c) {
  digitalWrite(_dc, LOW);
  digitalWrite(_cs, LOW);
  SPI.transfer(c);
  digitalWrite(_cs, HIGH);
}

/*****************/
/* digit display */
/*****************/
#define MAX_CHAR_IN_LINE 7

void ScreenDigit::display(double value) {

  /***************/
  /* build digit */
  /***************/

  /* to store the characters to be displayed */
  uint8_t digitCharacters[MAX_CHAR_IN_LINE];
  uint8_t digitSize = 0;

  /* compute the total digit width in pixels */
  digit.begin(value);
  uint8_t digitWidth = digit.size(VARIOSCREEN_DIGIT_WIDTH, VARIOSCREEN_DIGIT_WIDTH, VARIOSCREEN_DOT_WIDTH);

  /* get characters */
  while( digit.availiable() ) {
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
	displayCharacter(varioscreenPlus, VARIOSCREEN_DIGIT_WIDTH, line);
	break;

      case '-':
	displayCharacter(varioscreenMinus, VARIOSCREEN_DIGIT_WIDTH, line);
	break;

      case '.':
	displayCharacter(varioscreenDot, VARIOSCREEN_DOT_WIDTH, line);
	break;

      default:
	displayCharacter(varioscreenDigit[c - '0'], VARIOSCREEN_DIGIT_WIDTH, line);
	break;
      }
    }
    
    screen.endDisplay();
  }
  
  screen.flush();
}


void ScreenDigit::displayCharacter(const uint8_t* pointer, uint8_t width, uint8_t line) {
  for(int i = 0; i<width; i++) {
    screen.display( pgm_read_byte( &pointer[i + line * width] ) );
  }
}


void displayElement(VarioScreen& screen, const uint8_t* pointer, uint8_t posX, uint8_t posY, uint8_t width, uint8_t height) {
  for(int line=0; line<height; line++) {
    screen.beginDisplay(posX, posY + line);
    for( int i=0; i<width; i++ ) {
      screen.display( pgm_read_byte( &pointer[i + line * width] ) );
    }
    screen.endDisplay();
  }
  screen.flush();
}


void MSUnit::display(uint8_t posX, uint8_t posY) {
  /* display init */
  displayElement(screen, varioscreenMS, posX, posY, VARIOSCREEN_MS_WIDTH, VARIOSCREEN_FONT_HEIGHT);
}

void MUnit::display(uint8_t posX, uint8_t posY) {
  /* display init */
  displayElement(screen, varioscreenM, posX, posY, VARIOSCREEN_M_WIDTH, VARIOSCREEN_FONT_HEIGHT);
}

void KMHUnit::display(uint8_t posX, uint8_t posY) {
  /* display init */
  displayElement(screen, varioscreenKMH, posX, posY, VARIOSCREEN_KMH_WIDTH, VARIOSCREEN_FONT_HEIGHT);
}


void GRUnit::display(uint8_t posX, uint8_t posY) {
  /* display init */
  displayElement(screen, varioscreenGR, posX, posY, VARIOSCREEN_GR_WIDTH, VARIOSCREEN_FONT_HEIGHT);
}

