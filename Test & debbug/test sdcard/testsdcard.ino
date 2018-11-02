#include <Arduino.h>
#include <I2Cdev.h>
#include <ms5611.h>
#include <vertaccel.h>
#include <EEPROM.h>
#include <inv_mpu.h>
#include <avr/pgmspace.h>
#include <toneAC.h>
#include <FirmwareUpdater.h>
#include <IGCSentence.h>
#include <digit.h>

#include <SdCard.h>
#include <LightFat16.h>
#include <string.h>

#define BEEP_FREQ 800

#define SDCARD_CS_PIN 14

lightfat16 file;

#if F_CPU >= 16000000L
#define VARIOSCREEN_SPEED SPI_CLOCK_DIV4
#define SDCARD_SPEED SPI_CLOCK_DIV4
#else
#define VARIOSCREEN_SPEED SPI_CLOCK_DIV2
#define SDCARD_SPEED SPI_CLOCK_DIV2
#endif //CPU_FREQ

#define SDCARD_STATE_INITIAL 0
#define SDCARD_STATE_INITIALIZED 1
#define SDCARD_STATE_READY 2
#define SDCARD_STATE_ERROR -1
int8_t sdcardState = SDCARD_STATE_INITIAL;

const byte BUFFER_SIZE = 64;
 
/* Variables d'exemple qui seront chargé depuis le fichier de configuration */
int sensorValue;


void setup() {
  
  /* wait a little */
  delay(1000);
    
  /* launch firmware update if needed */
  Fastwire::setup(400,0);
  vertaccel_init();
  if( firmwareUpdateCond() ) {
   firmwareUpdate();
  }
  
    /* Déclare le buffer qui stockera une ligne du fichier, ainsi que les deux pointeurs key et value */
  char buffer[BUFFER_SIZE], *key, *value;
 
  /* Déclare l'itérateur et le compteur de lignes */
  byte i, buffer_lenght, line_counter = 0;
 
  /* Initialise la carte SD *
  pinMode(SDCARD_CS_PIN, OUTPUT);
  if (!SD.begin(4)) { // Gère les erreurs
    toneAC(BEEP_FREQ);
    delay(200);
    toneAC(0);
    for(;;);
  }*/
  
  if( file.init(SDCARD_CS_PIN, SDCARD_SPEED) >= 0 ) {
    sdcardState = SDCARD_STATE_INITIALIZED;  //useless to set error
  }
  else {
    toneAC(BEEP_FREQ);
    delay(200);
    toneAC(0);
    for(;;);
  }

  /* Ouvre le  fichier de configuration */
  
      delay(1000);


      /* build date */
    uint8_t dateChar[8]; //two bytes are used for incrementing number on filename
    for(int i=0; i<6; i++) {
      dateChar[i] = 'A';
    }

    /* create file */    
  if( file.begin((char*)dateChar, 8) >= 0 ) {
      sdcardState = SDCARD_STATE_READY;
    }
  else {
    toneAC(BEEP_FREQ);
    delay(200);
    toneAC(0);
    for(;;);
    }
 
     delay(1000);

   for (int i=1;i<500;i++) {
      file.write( 'A' );
    }

/*    for(i=0;i<100;i++){
        sensorValue = analogRead(A2);
        config_file.println(sensorValue);
        delay(100);
    }*/

    delay(1000);
    
    for( int i = 0; i<3; i++) {
      toneAC(BEEP_FREQ);
      delay(200);
      toneAC(0);
      delay(200);
    }
    
}

void loop() {
  

}
