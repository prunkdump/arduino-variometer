/*
 * List files in root directory.
 */
#include <Fat16.h>
#include <Fat16util.h>

const uint8_t CHIP_SELECT = SS;  // SD card chip select pin.
SdCard card;

// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))

void error_P(const char* str) {
  PgmPrint("error: ");
  SerialPrintln_P(str);
  if (card.errorCode) {
    PgmPrint("SD error: ");
    Serial.println(card.errorCode, HEX);
  }
  while(1);
}

void setup() {
  Serial.begin(9600);
  PgmPrintln("Type any character to start");
  while (!Serial.available());
  
  PgmPrint("Free RAM: ");
  Serial.println(FreeRam());  
 
  if (!card.begin(CHIP_SELECT)) error("card.begin");
  
  if (!Fat16::init(&card)) error("Fat16::init");
  
  Serial.println();
  
  PgmPrintln("Name          Modify Date/Time    Size");
  
  Fat16::ls(LS_DATE | LS_SIZE);
}

void loop() { }
