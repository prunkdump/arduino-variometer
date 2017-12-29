#include <Arduino.h>
#include <GenClock_zero.h>
#include <VarioSettings.h>
#include <SerialNmea_zero.h>
#include <toneAC_zero.h>
#include <NmeaParser.h>
#include <IGCSentence.h>

NmeaParser nmeaParser;

#ifdef HAVE_BLUETOOTH
boolean lastSentence = false;
#endif //HAVE_BLUETOOTH

unsigned long RMCSentenceTimestamp; //for the speed filter
double RMCSentenceCurrentAlti; //for the speed filter
unsigned long speedFilterTimestamps[VARIOMETER_SPEED_FILTER_SIZE];
double speedFilterSpeedValues[VARIOMETER_SPEED_FILTER_SIZE];
double speedFilterAltiValues[VARIOMETER_SPEED_FILTER_SIZE];
int8_t speedFilterPos = 0;


IGCHeader header;
IGCSentence igc;

void setup() {
  
  /* init tone */
   toneAC_init();

  /* init serial for debug */
  Serial.begin(9600);
  while (!Serial) { ;}

  /* init serial nmea */
 Serial1.begin(9600);
 serialNmea.begin(GPS_BLUETOOTH_BAUDS, true);
 
  Serial.println("Setup");
}

void loop() {


if( serialNmea.lockGGA() ) {
      nmeaParser.beginGGA();
      /* start to write IGC B frames */

    /* parse if needed */
    if( nmeaParser.isParsing() ) {
      while( nmeaParser.isParsing() ) {
        uint8_t c = serialNmea.read();
        
        /* parse sentence */        
        nmeaParser.feed( c );

        /* if GGA, convert to IGC and write to sdcard */
        if( nmeaParser.isParsingGGA() ) {
          igc.feed(c);
          while( igc.available() ) {           
 //           Serial.print( igc.get() );
          }
 //         Serial.println("");
        }
      }

      serialNmea.release();    
    }
  }


  /* beep after GGA */
/*  if( serialNmea.lockGGA() ) {
    serialNmea.release();

    toneAC(1000);

    serialNmea.write('H');
    serialNmea.write('e');
    serialNmea.write('l');
    serialNmea.write('l');
    serialNmea.write('o');
    serialNmea.write('!');
    serialNmea.write('\r');
    serialNmea.write('\n');

    delay(200);
    toneAC(0);
  }*/
}
