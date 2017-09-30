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

FlashStorage(setVolume, uint8_t);

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
void VarioSettings::writeSDSettings() {
  // Delete the old One
  SD.remove(FileName);
  // Create new one
  myFile = SD.open(FileName, FILE_WRITE);
  // writing in the file works just like regular print()/println() function
  myFile.print("[");
  myFile.print("exINT=");
  myFile.print(exINT);
  myFile.println("]");
  myFile.print("[");
  myFile.print("exFloat=");
  myFile.print(exFloat,5);
  myFile.println("]");
  myFile.print("[");
  myFile.print("exBoolean=");
  myFile.print(exBoolean);
  myFile.println("]");
  myFile.print("[");
  myFile.print("exLong=");
  myFile.print(exLong);
  myFile.println("]");
  // close the file:
  myFile.close();
  //Serial.println("Writing done.");
}

uint8_t VarioSettings::soundSettingRead(void) {
  uint8_t TmpValue;	
  TmpValue=setVolume.read();
#ifdef PROG_DEBUG
  Serial.print("Read sound volume : ");
  Serial.println(TmpValue);
#endif //PRO_DEBBUG

  if ((TmpValue<0) || (TmpValue>10)) {TmpValue=5;}
  return TmpValue;
}

void VarioSettings::soundSettingWrite(uint8_t volume) {
#ifdef PROG_DEBUG
  Serial.print("Write sound volume : ");
  Serial.println(volume);
#endif //PRO_DEBBUG

  setVolume.write(volume);	
}

void Statistic::setTime(int8_t* timeValue) {

  for(uint8_t i = 0; i<3; i++) {
    time[i] = timeValue[i];
  }
}

