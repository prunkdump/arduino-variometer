// Demo of fgets function to read lines from a file.
#include <Fat16.h>
#include <Fat16util.h> // use functions to print strings from flash memory

const uint8_t CHIP_SELECT = SS;  // SD chip select pin.

SdCard card;
Fat16 file;

// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))
//------------------------------------------------------------------------------
void error_P(const char* str) {
  PgmPrint("error: ");
  SerialPrintln_P(str);
  if (card.errorCode) {
    PgmPrint("SD error: ");
    Serial.println(card.errorCode, HEX);
  }
  while(1);
}
//------------------------------------------------------------------------------
void demoFgets() {
  char line[25];
  int n;
  // open test file
  if (!file.open("FGETS.TXT", O_READ)) {
    error("demoFgets");
  }
  PgmPrintln(
    "Lines with '>' end with a '\\n' character\r\n"
    "Lines with '#' do not end with a '\\n' character\r\n");
    
  // read lines from the file
  while ((n = file.fgets(line, sizeof(line))) > 0) {
    if (line[n - 1] == '\n') {
      // remove LF
      line[n - 1] = 0;
      Serial.write('>');
    } else {
      Serial.write('#');
    }
    Serial.println(line);
  }
}
//------------------------------------------------------------------------------
void makeTestFile() {
  // create or open test file
  if(!file.open("FGETS.TXT", O_WRITE | O_CREAT | O_TRUNC)) {
    error("MakeTestFile");
  }
  // write test file
  file.write_P(PSTR(
    "Line with CRLF\r\n"
    "Line with only LF\n"
    "Long line that will require an extra read\n"
    "\n"  // empty line
    "Line at EOF without NL"
  ));
  file.close();
}
//------------------------------------------------------------------------------
void setup(void) {
  Serial.begin(9600);
  while (!Serial) {}  // Wait for Leonardo

  PgmPrint("Type any character to start\n");
  while (Serial.read() <= 0) {}
  delay(400);  // catch Due reset problem
  // initialize the SD card
  if (!card.begin(CHIP_SELECT)) error("card.begin");
  
  // initialize a FAT16 volume
  if (!Fat16::init(&card)) error("Fat16::init");

  makeTestFile();
  
  demoFgets();
  
  PgmPrintln("\nDone");
}
void loop(void) {}