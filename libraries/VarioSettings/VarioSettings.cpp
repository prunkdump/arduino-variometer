/*
  SD card read/write

 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

 */

#include <VarioSettings.h>
#include <Arduino.h>
#include <FlashAsEEPROM.h>


boolean VarioSettings::initSettings() {
    if (!SD.begin(SDCARD_CS_PIN)) {
#ifdef IMU_DEBUG
      Serial.println("initialization failed!");
#endif //IMU_DEBUG
      return false;
    }
  return true;
}

boolean VarioSettings::readSDSettings(){
  char character;
  String settingName;
  String settingValue;
  myFile = SD.open(FileName);
  if (myFile) {
    while (myFile.available()) {
      character = myFile.read();
      while((myFile.available()) && (character != '[')){
        character = myFile.read();
      }
      character = myFile.read();
      while((myFile.available()) && (character != '=')){
        settingName = settingName + character;
        character = myFile.read();
      }
      character = myFile.read();
      while((myFile.available()) && (character != ']')){
        settingValue = settingValue + character;
        character = myFile.read();
      }
      
      if(character == ']'){
 
#ifdef IMU_DEBUG

        //Debuuging Printing
        Serial.print("Name:");
        Serial.println(settingName);
        Serial.print("Value :");
        Serial.println(settingValue);
#endif //IMU_DEBUG

        // Apply the value to the parameter
        applySetting(settingName,settingValue);
        // Reset Strings
        settingName = "";
        settingValue = "";
      }
    }
 
    // close the file:
    myFile.close();
	return true;
  } else {
   // if the file didn't open, print an error:
#ifdef IMU_DEBUG
   Serial.println("error opening settings.txt");
#endif //IMU_DEBUG   
   return false;
  }
}
 
 /* Apply the value to the parameter by searching for the parameter name
 Using String.toInt(); for Integers
 toFloat(string); for Float
 toBoolean(string); for Boolean
 toLong(string); for Long
 */
 void VarioSettings::applySetting(String settingName, String settingValue) {
 
   if (settingName == "VARIOMETER_PILOT_NAME") {
#ifdef IMU_DEBUG
       Serial.println("Model du vario : " + settingValue);
#endif //IMU_DEBUG
       VARIOMETER_PILOT_NAME=settingValue;
   }
#ifdef IMU_DEBUG  
   else if(settingName == "exINT") {
     exINT=settingValue.toInt();
   }
   else if(settingName == "exFloat") {
     exFloat=toFloat(settingValue);
   }
   else if(settingName == "exBoolean") {
     exBoolean=toBoolean(settingValue);
   }
   else if(settingName == "exLong") {
     exLong=toLong(settingValue);
   }
#endif //IMU_DEBUG   
    else if(settingName == "VARIOMETER_GLIDER_NAME") {
     VARIOMETER_GLIDER_NAME = settingValue;
   }
   else if(settingName == "VARIOMETER_BASE_PAGE_DURATION") {
  /* the duration of the two screen pages in milliseconds */
     VARIOMETER_BASE_PAGE_DURATION = settingValue.toInt();
   }
   else if(settingName == "VARIOMETER_ALTERNATE_PAGE_DURATION") {
	 VARIOMETER_ALTERNATE_PAGE_DURATION=settingValue.toInt();
   }
   else if(settingName == "VARIOMETER_BEEP_VOLUME") {
//	 VARIOMETER_BEEP_VOLUME=settingValue.toInt();
	 VARIOMETER_BEEP_VOLUME=soundSettingRead();
   }
   
  /* The variometer react like this according to vertical speed in m/s :        */
  /* (near climbing beep is not enabled by default)                             */
  /*                                                                            */
  /* <--LOW-BEEP--|------SILENT------|--NEAR-CLIMBING-BEEP--|--CLIMBING-BEEP--> */
  /*              |                  |                      |                   */
  /*           SINKING         CLIMBING-SENSITIVITY      CLIMBING               */
   
   
   else if(settingName == "VARIOMETER_SINKING_THRESHOLD") {
	 VARIOMETER_SINKING_THRESHOLD=toFloat(settingValue);
   }
   else if(settingName == "VARIOMETER_CLIMBING_THRESHOLD") {
	 VARIOMETER_CLIMBING_THRESHOLD=toFloat(settingValue);
   }
   else if(settingName == "VARIOMETER_NEAR_CLIMBING_SENSITIVITY") {
	 VARIOMETER_NEAR_CLIMBING_SENSITIVITY=toFloat(settingValue);
   }
   else if(settingName == "VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM") {
	     /* The near climbing alarm : signal that you enter or exit the near climbing zone */
     VARIOMETER_ENABLE_NEAR_CLIMBING_ALARM=toBoolean(settingValue);
   }
   else if(settingName == "VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP") {
	     /* The near climbing beep : beep when you are in near climbing zone               */
     VARIOMETER_ENABLE_NEAR_CLIMBING_BEEP=toBoolean(settingValue);
   }
   else if(settingName == "FLIGHT_START_MIN_TIMESTAMP") {
	     /* Flight start detection conditions :                      */
        /* -> Minimum time after poweron in milliseconds            */
     FLIGHT_START_MIN_TIMESTAMP=toFloat(settingValue);
   }
    else if(settingName == "FLIGHT_START_VARIO_LOW_THRESHOLD") {
		  /* Flight start detection conditions :                      */
		  /* -> Minimum vertical velocity in m/s (low/high threshold) */
     FLIGHT_START_VARIO_LOW_THRESHOLD=toFloat(settingValue);
   }
   else if(settingName == "FLIGHT_START_VARIO_HIGH_THRESHOLD") {
	     /* Flight start detection conditions :                      */
  /* -> Minimum vertical velocity in m/s (low/high threshold) */
     FLIGHT_START_VARIO_HIGH_THRESHOLD=toFloat(settingValue);
   }
   else if(settingName == "FLIGHT_START_MIN_SPEED") {
  /* Flight start detection conditions :                      */
 /* -> Minimum ground speed in km/h                          */
	 FLIGHT_START_MIN_SPEED=toFloat(settingValue);
   }
   else if(settingName == "VARIOMETER_RECORD_WHEN_FLIGHT_START") {
  /* GPS track recording on SD card starting condition :  */ 
  /* -> As soon as possible (GPS fix)                     */
  /* -> When flight start is detected                     */
    VARIOMETER_RECORD_WHEN_FLIGHT_START=toBoolean(settingValue);
   }
 /*  else if(settingName == "VARIOMETER_SENT_LXNAV_SENTENCE") {
  /* What type of vario NMEA sentence is sent by bluetooth. */
  /* Possible values are :                                  */
  /*  - VARIOMETER_SENT_LXNAV_SENTENCE                      */
  /*  - VARIOMETER_SENT_LK8000_SENTENCE                     *
     VARIOMETER_SENT_LXNAV_SENTENCE=toBoolean(settingValue);
   }*/
   else if(settingName == "ALARM_SDCARD") {
     /* Alarm */
     /* Alarm SDCARD not insert */
     ALARM_SDCARD=toBoolean(settingValue);
   }
   else if(settingName == "ALARM_GPSFIX") {
	 /* Alarm GPS Fix */
     ALARM_GPSFIX=toBoolean(settingValue);
   }
   else if(settingName == "ALARM_FLYBEGIN") {
	 /* Alarm Fly begin */
     ALARM_FLYBEGIN=toBoolean(settingValue);
   }
    else if(settingName == "KF_ZMEAS_VARIANCE") {
	 // Kalman filter configuration
	 KF_ZMEAS_VARIANCE=toFloat(settingValue);
   }
   else if(settingName == "KF_ZACCEL_VARIANCE") {
	   // Kalman filter configuration
     KF_ZACCEL_VARIANCE=toFloat(settingValue);
   }
   else if(settingName == "KF_ACCELBIAS_VARIANCE") {
	   // Kalman filter configuration
     KF_ACCELBIAS_VARIANCE=toFloat(settingValue);
   }
    else if(settingName == "SLEEP_TIMEOUT_SECONDS") {
		// Power-down timeout. Here we power down if the
		// vario does not see any climb or sink rate more than
		// 50cm/sec, for 20 minutes.
     SLEEP_TIMEOUT_SECONDS=settingValue.toInt(); // 20 minutes
   }
   else if(settingName == "SLEEP_THRESHOLD_CPS") {
	    // Power-down timeout. Here we power down if the
		// vario does not see any climb or sink rate more than
		// 50cm/sec, for 20 minutes.
     SLEEP_THRESHOLD_CPS=settingValue.toInt();
   }
   // vario thresholds in cm/sec for generating different
// audio tones. Between the sink threshold and the zero threshold,
// the vario is quiet
/*                                                                            */
/* <--LOW-BEEP--|------SILENT------|--NEAR-CLIMBING-BEEP--|--CLIMBING-BEEP--> */
/*              |                  |                      |                   */
/*             SINK              ZERO                   CLIMB                 */
   else if(settingName == "CLIMB_THRESHOLD") {
	 CLIMB_THRESHOLD=settingValue.toInt();
   }
   else if(settingName == "ZERO_THRESHOLD") {
	 ZERO_THRESHOLD=settingValue.toInt();
   }
   else if(settingName == "SINK_THRESHOLD") {
     SINK_THRESHOLD=settingValue.toInt();
   }
   else if(settingName == "VARIO_MAX_FREQHZ") {
// change these parameters based on the frequency bandwidth of the speaker
     VARIO_MAX_FREQHZ=settingValue.toInt();
   }
   else if(settingName == "VARIO_XOVER_FREQHZ") {
// change these parameters based on the frequency bandwidth of the speaker
	 VARIO_XOVER_FREQHZ=settingValue.toInt();
   }
   else if(settingName == "VARIO_MIN_FREQHZ") {
 // change these parameters based on the frequency bandwidth of the speaker
    VARIO_MIN_FREQHZ=settingValue.toInt();
   }
   else if(settingName == "VARIO_SINK_FREQHZ") {
     VARIO_SINK_FREQHZ=settingValue.toInt();
   }
   else if(settingName == "VARIO_TICK_FREQHZ") {
	 VARIO_TICK_FREQHZ=settingValue.toInt();
   }
   else if(settingName == "BATTERY_TONE_FREQHZ") {
 // audio feedback tones
    BATTERY_TONE_FREQHZ=settingValue.toInt();
   }
   else if(settingName == "CALIB_TONE_FREQHZ") {
 // audio feedback tones
    CALIB_TONE_FREQHZ=settingValue.toInt();
   }
   else if(settingName == "MPU9250_ERROR_TONE_FREQHZ") {
 // audio feedback tones
    MPU9250_ERROR_TONE_FREQHZ=settingValue.toInt();
   }
   else if(settingName == "MS5611_ERROR_TONE_FREQHZ") {
 // audio feedback tones
    MS5611_ERROR_TONE_FREQHZ=settingValue.toInt();
   }
   else if(settingName == "SDCARD_ERROR_TONE_FREQHZ") {
 // audio feedback tones
    SDCARD_ERROR_TONE_FREQHZ=settingValue.toInt();
   }  
   else if(settingName == "SDCARD_ERROR_TONE_FREQHZ") {
 // audio feedback tones
    BEEP_FREQ=settingValue.toInt();
   }  
   else {       
   }  
}
 
 // converting string to Float
float VarioSettings::toFloat(String settingValue){
   char floatbuf[settingValue.length()+1];
   settingValue.toCharArray(floatbuf, sizeof(floatbuf));
   float f = atof(floatbuf);
   return f;
}
 
long VarioSettings::toLong(String settingValue){
   char longbuf[settingValue.length()+1];
   settingValue.toCharArray(longbuf, sizeof(longbuf));
   long l = atol(longbuf);
   return l;
}
 
 
 // Converting String to integer and then to boolean
 // 1 = true
 // 0 = false
boolean VarioSettings::toBoolean(String settingValue) {
  if(settingValue.toInt()==1){
    return true;
  } else {
    return false;
  }
}
 
 // Writes A Configuration file DEBBUG
void VarioSettings::writeFlashSDSettings() {
	
	
File flashfile;
  
  // create a new file
  char filename[] = "FLASH.TXT";
  if (! SD.exists(filename)) {
    // only open a new file if it doesn't exist
    flashfile = SD.open(filename, FILE_WRITE);
  }
  else {
	SD.remove(filename);
    flashfile = SD.open(filename, FILE_WRITE);
  }
  
 if (! flashfile) {
    // if the file didn't open, print an error:
 #ifdef IMU_DEBUG
   Serial.println("error opening test.txt");
    Serial.println("couldnt create file");
 #endif //IMU_DEBUG
  }
  else {
    Serial.print("Logging to: ");
    Serial.println(filename);

    flashfile.println("This is a test file");
    // if ECHO_TO_SERIAL
    Serial.println("This is a test file");
  
  
    if (!flashfile.println() ) {
      Serial.println("error with write header");
    }
 
    flashfile.print("Hello World");
    Serial.println("Hello World");

    if(!flashfile.println()){
      Serial.println("SD printing failed");
    }
  
    flashfile.flush();
    flashfile.close();
	
/*  // Delete the old One
 // SD.remove(FileFlashName);
  // Create new one
  myFile2 = SD.open(FileFlashName, FILE_WRITE);
  if (myFile2) {
     // writing in the file works just like regular print()/println() function
    myFile2.print("[");
    myFile2.print("SOUND=");
    myFile2.print(VARIOMETER_BEEP_VOLUME);
    myFile2.println("]");
    myFile2.print("[");
    myFile2.print("ACCELCALX=");
    myFile2.print(ACCELCALX,5);
    myFile2.println("]");
    myFile2.print("[");
    myFile2.print("ACCELCALY=");
    myFile2.print(ACCELCALY,5);
    myFile2.println("]");
    myFile2.print("[");
    myFile2.print("ACCELCALZ=");
    myFile2.print(ACCELCALZ,5);
    myFile2.println("]");
  
 /* myFile.print("[");
  myFile.print("exBoolean=");
  myFile.print(exBoolean);
  myFile.println("]");
  myFile.print("[");
  myFile.print("exLong=");
  myFile.print(exLong);
  myFile.println("]");*/
  // close the file:
    /*myFile.flush();
    myFile2.close();*/

 }
 
  
#ifdef IMU_DEBUG
        //Debuuging Printing
    Serial.println("Write File SD");
	Serial.println("Writing done.");
#endif //IMU_DEBUG

}

boolean VarioSettings::readFlashSDSettings(){
/*  char character;
  String settingName;
  String settingValue;
  myFile = SD.open(FileFlashName);
  if (myFile) {
    while (myFile.available()) {
      character = myFile.read();
      while((myFile.available()) && (character != '[')){
        character = myFile.read();
      }
      character = myFile.read();
      while((myFile.available()) && (character != '=')){
        settingName = settingName + character;
        character = myFile.read();
      }
      character = myFile.read();
      while((myFile.available()) && (character != ']')){
        settingValue = settingValue + character;
        character = myFile.read();
      }
      
      if(character == ']'){
 
#ifdef IMU_DEBUG

        //Debuuging Printing
        Serial.print("Name:");
        Serial.println(settingName);
        Serial.print("Value :");
        Serial.println(settingValue);
#endif //IMU_DEBUG

        // Apply the value to the parameter
        applyFlashSetting(settingName,settingValue);
        // Reset Strings
        settingName = "";
        settingValue = "";
      }
    }
 
    // close the file:
    myFile.close();
	return true;
  } else {
   // if the file didn't open, print an error:
#ifdef IMU_DEBUG
   Serial.print("error opening : ");
   Serial.println(FileFlashName);
#endif //IMU_DEBUG   
   return false;
  }*/
  
File flashfile;
  
  char filename[] = "FLASH.TXT";
  if (! SD.exists(filename)) {
#ifdef IMU_DEBUG
     Serial.print("error opening : ");
     Serial.println(FileFlashName);
#endif //IMU_DEBUG   
    // only open a new file if it doesn't exist
  }
  else {
    flashfile = SD.open(filename);
  }
  
 if (! flashfile) {
    Serial.println("couldnt open file");
  }
  else {
    Serial.print("Logging to: ");
    Serial.println(filename);

    // read from the file until there's nothing else in it:
    while (flashfile.available()) {
      Serial.write(flashfile.read());
    }
    // close the file:
    flashfile.close();
  }
//}
}
 
 /* Apply the value to the parameter by searching for the parameter name
 Using String.toInt(); for Integers
 toFloat(string); for Float
 toBoolean(string); for Boolean
 toLong(string); for Long
 */
 void VarioSettings::applyFlashSetting(String settingName, String settingValue) {
 
   if (settingName == "SOUND") {
#ifdef IMU_DEBUG
       Serial.println("Sound Read File : " + settingValue);
#endif //IMU_DEBUG
     VARIOMETER_BEEP_VOLUME = settingValue.toInt();
   }
   else if(settingName == "ACCELCALX") {
     ACCELCALX=toFloat(settingValue);
   }
   else if(settingName == "ACCELCALY") {
     ACCELCALY=toFloat(settingValue);
   }
   else if(settingName == "ACCELCALZ") {
     ACCELCALZ=toFloat(settingValue);
   }
 }   
   
   
uint8_t VarioSettings::soundSettingRead(void) {
  /* check tag */
  uint16_t eepromTag;
  uint8_t tmpValue;
  
  if (!EEPROM.isValid()) {
    Serial.println("EEPROM is empty, writing some example data:");
	VARIOMETER_BEEP_VOLUME=0;
	readFlashSDSettings();
	if (VARIOMETER_BEEP_VOLUME==0) { tmpValue = 5; }
  } else {
	  
    eepromTag = EEPROM.read(SOUND_EPROM_ADDR);
    eepromTag <<= 8;
    eepromTag += EEPROM.read(SOUND_EPROM_ADDR + 0x01);
  
    uint8_t TmpValue;
    if( eepromTag != SOUND_EPROM_TAG ) { 
	  VARIOMETER_BEEP_VOLUME=0;
	  readFlashSDSettings();
	  if (VARIOMETER_BEEP_VOLUME==0) { tmpValue = 5; }
    } else {
      /* read calibration settings */
      tmpValue =  EEPROM.read(SOUND_EPROM_ADDR + 0x02);
#ifdef PROG_DEBUG
      Serial.print("Read sound value : ");
      Serial.println(tmpValue);
#endif //PRO_DEBBUG

    }
  }

#ifdef PROG_DEBUG
  Serial.print("Sound value : ");
  Serial.println(tmpValue);
#endif //PRO_DEBBUG

  if ((tmpValue<0) || (tmpValue>10)) {tmpValue=5;}
  return tmpValue;
}

void VarioSettings::soundSettingWrite(uint8_t volume) {
#ifdef PROG_DEBUG
  Serial.print("Write sound volume : ");
  Serial.println(volume);
#endif //PRO_DEBBUG

  /* write tag */
  uint16_t eepromTag = SOUND_EPROM_TAG;
  EEPROM.write(SOUND_EPROM_ADDR, (eepromTag>>8) & 0xff);
  EEPROM.write(SOUND_EPROM_ADDR + 0x01, eepromTag & 0xff);

  EEPROM.write(SOUND_EPROM_ADDR + 0x02 , volume);
  
  EEPROM.commit();
  
  VARIOMETER_BEEP_VOLUME=volume;
  writeFlashSDSettings();
}

void Statistic::setTime(int8_t* timeValue) {

  for(uint8_t i = 0; i<3; i++) {
    time[i] = timeValue[i];
  }
}

int8_t* Statistic::getTime(void) {
  return time;	
}

int8_t* Statistic::getTime(int8_t* timeValue) {
  for(uint8_t i = 0; i<3; i++) {
    timeValue[i] = time[i];
  }	

  return time;	
}

void Statistic::setDuration(int8_t* durationValue) {
  for(uint8_t i = 0; i<3; i++) {
    duration[i] = durationValue[i];
  }	
}
   
int8_t* Statistic::getDuration(void) {
  return duration;
}

int8_t* Statistic::getDuration(int8_t* durationValue) {
  for(uint8_t i = 0; i<3; i++) {
    durationValue[i] = duration[i];
  }	

  return duration;	
}   

void Statistic::setAlti(double alti) {
  if (currentAlti == 0) {
	 currentAlti = alti;
	 altiDeco = alti;
	 maxAlti = alti;
	 minAlti = alti;
  }

  if (alti > maxAlti) {maxAlti = alti;}
  else if (alti < minAlti) {minAlti = alti;}
}
   
double Statistic::getMaxAlti(void) {
  return maxAlti;	
}
   
double Statistic::getMinAlti(void) {
  return minAlti;	
}
   
void Statistic::setVario(double vario) {
  if (vario > maxVario) {maxVario = vario;}
  else if (vario < minVario) {minVario = vario;}	
}
   
double Statistic::getMaxVario(void) {
  return maxVario;		
}
   
double Statistic::getMinVario(void) {
  return minVario;		
}
   
void Statistic::setSpeed(double speed) {
  if (speed > maxSpeed) {maxSpeed = speed;}
  else if (speed < minSpeed) {minSpeed = speed;}		
}
   
double Statistic::getMaxSpeed(void) {
  return maxSpeed;			
}
   
double Statistic::getMinSpeed(void) {
  return minSpeed;				
}
   
double Statistic::getAltiDeco(void) {
  return currentAlti;	
}
   
double Statistic::getGain(void) {
  return maxAlti - currentAlti;	
}

