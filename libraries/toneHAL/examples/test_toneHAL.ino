#include <toneHAL.h>
#if defined(ESP8266) 
#elif defined(ESP32)
#elif defined(ARDUINO_AVR_PRO)
#include "toneHAL_PRO.h"
#elif defined(ARDUINO_ARCH_SAMD)
#include "toneHAL_M0.h"
#endif

#include "Debug.h"
#include "param.h"

#if defined(TONEDAC_EXTENDED)
#include <SPI.h>
//#include <SD.h>
#include "SdFat.h"
SdFat SD;

File myFile;

const char *filename = "music.wav";

#endif //TONEDAC_EXTENDED

ToneHAL toneHAL;

#define volumeDefault 5

//indicate sample rate here (use audacity to convert your wav)
const unsigned int sampleRateWav = 22050;

void setup() {
  // put your setup code here, to run once:
  SerialPort.begin(115200);
  while (!SerialPort) { ;}
  
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

#if defined(ARDUINO_ARCH_SAMD)

  //activation de l'ampli D
  digitalWrite(6, LOW);
  pinMode(6, OUTPUT);
  digitalWrite(6, HIGH);

#endif //ARDUINO_ARCH_SAMD

#if defined(TONEDAC_EXTENDED)

  SerialPort.print("Initializing SD card...");

  if (!SD.begin(SDCARD_CS_PIN)) {
    SerialPort.println("initialization failed!");
//    return;
  }
  else {
    SerialPort.println("initialization done.");

/*    myFile = SD.open("test.txt", FILE_WRITE);
    if (myFile) {
       
    }*/
  }
#endif //TONEDAC_EXTENDED
    
  toneHAL.init();

  SerialPort.print("Tone Sin volume = ");
  SerialPort.println(volumeDefault);

  SerialPort.println("Tone Sin");
  for(int i=1; i<=10; i++) {
    toneHAL.tone(1000*i,volumeDefault);
    delay(100);
  }
  toneHAL.noTone();
  delay(2000);
  
  SerialPort.println("Tone volume");
  for(int i=0; i<=10; i++) {
    toneHAL.tone(8000,i);
    delay(100);
  }
  toneHAL.noTone();
  delay(2000);
  
#if defined(TONEDAC)
  
  SerialPort.println("Tone Square");
  toneHAL.setWaveForm(WAVEFORM_SQUARE);
  for(int i=1; i<=10; i++) {
    toneHAL.tone(1000*i,volumeDefault);
    delay(100);
  }
  toneHAL.noTone();
  delay(2000);
   
  SerialPort.println("Tone Triangle");
  toneHAL.setWaveForm(WAVEFORM_TRIANGLE);
  for(int i=1; i<=10; i++) {
    toneHAL.tone(1000*i,volumeDefault);
    delay(100);
  }
  toneHAL.noTone();
  delay(2000); 

//  toneHAL.beginPlayWav(sampleRateWav);
//  toneHAL.playWav(filename);
//  SerialPort.println("Playing file.....");
#endif //TONEDAC
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  delay(200);
  if (SerialPort.available()) {
    char c = SerialPort.read();
    if (c = 'p') {
      for(int i=1; i<=10; i++) {
        toneHAL.tone(1000*i,volumeDefault);
        delay(100);
      }
      toneHAL.noTone();    
    }
  }
}


