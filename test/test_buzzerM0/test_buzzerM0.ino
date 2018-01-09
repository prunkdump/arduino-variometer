// CJMCU-117 MPU9250+MS5611 circuit interface
//
// VCC  VCC - BAT
// GND  GND
// SCL  D12 - SCL
// SDA  D11 - SDA
//
// LM9110 
// VCC   VCC - BAT
// PWM   A3, A4 PWM
//
// A1,A2 Switch 
// A5    Detection ON/OFF  
//
// A6    Detection de connection USB
// D6    Commande de l'alimentation des cartes
// D0    Reset command
// 
// E-Ink
// VCC   3.3V M0
// CS    D1
// BUSY  D3
// RST   D2
// DC    D7
// DIN   MOSI/D8
// CLK   SCK/D9
//
// GPS 
// VCC   VCC - BAT
// TX    D5 serialNmea  Pin 5     Sercom4
// RX    TX Serial1     Pin 14    Sercom5
//
// Bluetooth 
// VCC   VCC - BAT
// TX    RX Serial1      Pin 13   Sercom5
// RX    D4 serialNmea   Pin 4    Sercom4

//SERCOM 0 WIRE / I2C - SDA/SCL     - MPU
//SERCOM 1 SPI  - MISO-MOSI-SCK     - SCREEN
//SERCOM 2 SPI SD                   - SDCARD
//SERCOM 3 
//SERCOM 4 UART                     - GPS / BT
//SERCOM 5 UART   - SERIAL1         - GPS / BT

#include <SDU.h>    //FIRMWARE Update

#include <Arduino.h>
#include <SPI.h>
#include <VarioSettings.h>
#include <GenClock_zero.h>
#include <beeper.h>
#include <toneAC_zero.h>
#include <SD.h>

#include <Utility.h>

/*******************/
/* Version         */
/*******************/

#define VERSION 63
#define SUB_VERSION 0

/*******************/
/*     VERSION     */
/*    M0/SAMD21    */
/*                 */
/*    Historique   */
/*******************/
/* v 63.0     beta 1 version
 * 
 * v 63.0     beta 2 version
 *            version basée sur le code et les librairies M0 de PRUNKDUMP
 *            -Paramettres dans fichier TXT
 *            - Ecran I-Ink
 * v 63.0     beta 2.1 version           
 *            - add GPS Neo 8 
 *            - fix display bugs
 *            - add record Indicator, gps indicator
 * v 63.0     beta 2.2 version           
 *            - update display library
*******************/


#include <gnuvario.h>


#define MEASURE_DELAY 3000 

#define HIGH_BEEP_FREQ 1000.0
#define LOW_BEEP_FREQ 100.0
#define BASE_BEEP_DURATION 100.0

#ifdef HAVE_SPEAKER
//beeper beeper(VARIOMETER_SINKING_THRESHOLD, VARIOMETER_CLIMBING_THRESHOLD, VARIOMETER_NEAR_CLIMBING_SENSITIVITY, VARIOMETER_BEEP_VOLUME);
Beeper beeper(10);
#endif

#ifdef HAVE_SDCARD
File file;

VarioSettings GnuSettings;

#define SDCARD_STATE_INITIAL 0
#define SDCARD_STATE_INITIALIZED 1
#define SDCARD_STATE_READY 2
#define SDCARD_STATE_ERROR -1
int8_t sdcardState = SDCARD_STATE_INITIAL;

#endif //HAVE_SDCARD


/*-----------------*/
/*      SETUP      */
/*-----------------*/
void setup() {

/************************/
/*        Init Power    */
/************************/

//  statePower = HIGH;
  pinMode(VARIO_DETECT_USB, INPUT_PULLDOWN);

  pinMode(VARIO_PIN_ALIM, OUTPUT);
  digitalWrite(VARIO_PIN_ALIM, HIGH);   // turn on power cards )

  digitalWrite(VARIO_PIN_RST, HIGH);   // Hard Reset M0 )
  pinMode(VARIO_PIN_RST, OUTPUT);

  /*****************************/
  /* wait for devices power on */
  /*****************************/

 #ifdef PROG_DEBUG
  char tmpbuffer[50];

  Serial.begin(9600);
  while (!Serial) { ;}
  sprintf(tmpbuffer,"SAMD21 MPU9250 MS5611 VARIO compiled on %s at %s", __DATE__, __TIME__);
  Serial.println(tmpbuffer);
  Serial.flush();
#endif //PRO_DEBBUG

  /****************/
  /* init speaker */
  /****************/
#ifdef PROG_DEBUG
  Serial.println("Initializing ToneAC");
#endif //PRO_DEBBUG
  
#ifdef HAVE_SPEAKER
  toneAC_init();
#endif


  /****************/
  /* init SD Card */
  /****************/
  
#ifdef HAVE_SDCARD
#ifdef PROG_DEBUG
  Serial.println("Initializing SD card...");
#endif //PRO_DEBBUG

  if (GnuSettings.initSettings()) {
#ifdef PROG_DEBUG
   Serial.println("initialization done.");
#endif //PROG_DEBUG

   GnuSettings.readSDSettings();
 
#ifdef PROG_DEBUG
   //Debuuging Printing
 Serial.print("Pilot Name = ");
 Serial.println(GnuSettings.VARIOMETER_PILOT_NAME);
#endif //PROG_DEBUG

    sdcardState = SDCARD_STATE_INITIALIZED;  //useless to set error
  }
  else
  {
#ifdef HAVE_SPEAKER
    if (GnuSettings.ALARM_SDCARD) {
#ifdef PROG_DEBUG
      Serial.println("initialization failed!");
#endif //PROG_DEBUG

      indicateFaultSDCARD();
    }
#endif //HAVE_SPEAKER 
  }  
#endif //HAVE_SDCARD


/******************/
/*    init Audio  */
/******************/
#ifdef HAVE_SPEAKER
beeper.init(GnuSettings.VARIOMETER_SINKING_THRESHOLD, GnuSettings.VARIOMETER_CLIMBING_THRESHOLD, GnuSettings.VARIOMETER_NEAR_CLIMBING_SENSITIVITY, GnuSettings.VARIOMETER_BEEP_VOLUME);

#ifdef PROG_DEBUG
  sprintf(tmpbuffer,"Volume : %i", GnuSettings.VARIOMETER_BEEP_VOLUME);
  Serial.println(tmpbuffer);
  Serial.flush();
#endif //PROG_DEBUG

beeper.setVolume(GnuSettings.VARIOMETER_BEEP_VOLUME);
#endif
 

#ifdef PROG_DEBUG
  int tmpvolume = beeper.getVolume();
  beeper.setVolume(10);
#endif


#ifdef HAVE_SPEAKER

#ifdef PROG_DEBUG
  Serial.println("beeper vol = 10");
  Serial.flush();
#endif //PROG_DEBUG

  beeper.GenerateTone(GnuSettings.CALIB_TONE_FREQHZ, 3000);
    // allow 10 seconds for the unit to be placed in calibration position with the 
    // accelerometer +z pointing downwards. Indicate this delay with a series of short beeps
  for (int inx = 0; inx < 10; inx++) {
    delay(200); 
    beeper.GenerateTone(GnuSettings.CALIB_TONE_FREQHZ,50);
  }
#endif //HAVE_SPEAKER

#ifdef PROG_DEBUG
  Serial.println("signal beep vol = 10");
  Serial.flush();
#endif //PROG_DEBUG

  /* start beep */
    signalBeep(HIGH_BEEP_FREQ, BASE_BEEP_DURATION, 3);

#ifdef PROG_DEBUG
  sprintf(tmpbuffer,"Beeper vol = %i", GnuSettings.VARIOMETER_BEEP_VOLUME);
  Serial.println(tmpbuffer);
  Serial.flush();
#endif //PROG_DEBUG

#ifdef HAVE_SPEAKER
 beeper.setVolume(tmpvolume);
#endif //HAVE_SPEAKER
 

#ifdef HAVE_SPEAKER
  beeper.setVolume(GnuSettings.VARIOMETER_BEEP_VOLUME);
#else
  volLevel.setVolume(0);
#endif //HAVE_SPEAKER

#ifdef HAVE_SPEAKER
    beeper.GenerateTone(GnuSettings.CALIB_TONE_FREQHZ, 1000);
    delay(200); 
    beeper.GenerateTone(GnuSettings.CALIB_TONE_FREQHZ, 1000);
    delay(200); 
    beeper.GenerateTone(GnuSettings.CALIB_TONE_FREQHZ, 1000);
#endif //HAVE_SPEAKER

#ifdef PROG_DEBUG
  Serial.println("toneAc");
  Serial.flush();
#endif //PROG_DEBUG

toneAC(2000);
delay(1000);
noToneAC();  

}

/*----------------*/
/*      LOOP      */
/*----------------*/
void loop() {
}

