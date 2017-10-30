
#include <Utility.h>
#include <VarioSettings.h>
#include <Arduino.h>
#include <beeper.h>
#include <toneAC_Zero.h>

/**********************/
/* sensor objects */
/**********************/

/*Battery voltage * 10 */
int batteryVoltage(void) {
  int adcSample = 0;
  for (int inx = 0; inx < 4; inx++) {
    adcSample += analogRead(VOLTAGE_DIVISOR_PIN);
    delay(1);
    }
  adcSample /= 4;
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 4.3V):
  return (int) ((adcSample*4.3f*10.0f)/1023.0f + 0.5f); //  voltage x 10
  }  

// if imu calibration data in flash is corrupted, the accel and gyro biases are 
// set to 0, and this uncalibrated state is indicated with a sequence of alternating 
// low and high beeps.
void indicateUncalibratedAccelerometer() {
  for (int cnt = 0; cnt < 5; cnt++) {
#ifdef HAVE_SPEAKER
    beeper.GenerateTone(200,100); 
    beeper.GenerateTone(2000,100);
#endif //HAVE_SPEAKER
    }
  }

// "no-activity" power down is indicated with a series of descending
// tones. If you hear this, switch off the vario as there is still
// residual current draw from the circuit components  
void indicatePowerDown() {
#ifdef HAVE_SPEAKER
  beeper.GenerateTone(2000,1000); 
  beeper.GenerateTone(1000,1000);
  beeper.GenerateTone(500, 1000);
  beeper.GenerateTone(250, 1000);
#endif //HAVE_SPEAKER
  }

// problem with MS5611 calibration CRC, assume communication 
// error or bad device. Indicate with series of 10 high pitched beeps.
void indicateFaultMS5611() {
#ifdef HAVE_SPEAKER
  for (int cnt = 0; cnt < 10; cnt++) {
    beeper.GenerateTone(GnuSettings.MS5611_ERROR_TONE_FREQHZ,1000); 
    delay(100);
    }
#endif //HAVE_SPEAKER    
  }

// problem reading MPU9250 ID, assume communication 
// error or bad device. Indicate with series of 10 low pitched beeps.
void indicateFaultMPU9250() {
#ifdef HAVE_SPEAKER
  for (int cnt = 0; cnt < 10; cnt++) {
    beeper.GenerateTone(GnuSettings.MPU9250_ERROR_TONE_FREQHZ,1000); 
    delay(100);
    }
#endif //HAVE_SPEAKER    
  }


// problem SDCARD, assume communication 
// error or bad device. Indicate with series of 10 low pitched beeps.
void indicateFaultSDCARD() {
#ifdef HAVE_SPEAKER
  for (int cnt = 0; cnt < 10; cnt++) {
    beeper.GenerateTone(GnuSettings.SDCARD_ERROR_TONE_FREQHZ,1000); 
    delay(100);
    }
#endif //HAVE_SPEAKER
  }

/* make beeps */
void signalBeep(double freq, unsigned long duration, int count = 1) {

#ifdef HAVE_SPEAKER 
  toneAC(freq);
  delay(duration);
  toneAC(0.0);
  if( count > 1 ) {
    while( count > 1 ) {
      delay(duration);
      toneAC(freq);
      delay(duration);
      toneAC(0.0);
      count--;
    }
  }
#endif //HAVE_SPEAKER  
}

void powerDown() {
  // Mettre le SAMD21 en mode veille

#ifdef PROG_DEBUG
        Serial.print("Mise en veille");
        Serial.flush();
        delay(100);
#endif //PRO_DEBBUG
/*
   digitalWrite(VARIO_PIN_ALIM, LOW);   // turn on power cards )

//   detachInterrupt(digitalPinToInterrupt(VARIOPOWER_INT_PIN));
   detachInterrupt(digitalPinToInterrupt(VARIOBTN_LEFT_PIN));
   detachInterrupt(digitalPinToInterrupt(VARIOBTN_RIGHT_PIN));

   statePower = LOW;

//   nrgSave.begin(WAKE_EXT_INTERRUPT, VARIOPOWER_INT_PIN, POWERInterruptHandler);  //standby setup for external interrupts
  
  nrgSave.standby();  //now mcu goes in standby mode

 // rtc.standbyMode();

   digitalWrite(VARIO_PIN_RST, LOW);   // Hard Reset M0 )
   delay(500);

 //  NVIC_SystemReset();      // processor software reset 

*/     
#ifdef PROG_DEBUG
    Serial.println("Sortie du mode veille");
    Serial.flush();
    delay(100);
#endif //PRO_DEBBUG

 }  
 
 
 void Compteur::initTime() {
	timeNowUs = timePreviousUs = micros();
	}

void Compteur::updateTime(){
	timeNowUs = micros();
	imuTimeDeltaSecs = ((timeNowUs - timePreviousUs) / 1000000.0f);
	timePreviousUs = timeNowUs;
	}

bool Compteur::IsDelay(int delayCompteur) {
  boolean result = false;
  
  timeNowUs = micros();
  if ((timeNowUs - timePreviousUs) >= delayCompteur) result = true;

  return result;
}
	
#define cellfull 4.2
#define cellempty 3	
	
//--## Tableau de decharge LIPO	
//{{3.000, 0}, {3.053, 1}, {3.113, 2}, {3.174, 3}, {3.237, 4}, {3.300, 5}, {3.364, 6}, {3.427, 7}, {3.488, 8}, {3.547, 9}, {3.600, 10}, {3.621, 11}, {3.637, 12}, {3.649, 13}, {3.659, 14}, {3.668, 15}, {3.676, 16}, {3.683, 17}, {3.689, 18}, {3.695, 19}, {3.700, 20}, {3.706, 21}, {3.712, 22}, {3.717, 23}, {3.723, 24}, {3.728, 25}, {3.732, 26}, {3.737, 27}, {3.741, 28}, {3.746, 29}, {3.750, 30}, {3.754, 31}, {3.759, 32}, {3.763, 33}, {3.767, 34}, {3.771, 35}, {3.775, 36}, {3.779, 37}, {3.782, 38}, {3.786, 39}, {3.790, 40}, {3.794, 41}, {3.798, 42}, {3.802, 43}, {3.806, 44}, {3.810, 45}, {3.814, 46}, {3.818, 47}, {3.822, 48}, {3.826, 49}, {3.830, 50}, {3.834, 51}, {3.838, 52}, {3.842, 53}, {3.846, 54}, {3.850, 55}, {3.854, 56}, {3.858, 57}, {3.862, 58}, {3.866, 59}, {3.870, 60}, {3.875, 61}, {3.880, 62}, {3.885, 63}, {3.890, 64}, {3.895, 65}, {3.900, 66}, {3.905, 67}, {3.910, 68}, {3.915, 69}, {3.920, 70}, {3.924, 71}, {3.929, 72}, {3.933, 73}, {3.938, 74}, {3.942, 75}, {3.947, 76}, {3.952, 77}, {3.958, 78}, {3.963, 79}, {3.970, 80}, {3.982, 81}, {3.994, 82}, {4.007, 83}, {4.020, 84}, {4.033, 85}, {4.047, 86}, {4.060, 87}, {4.074, 88}, {4.087, 89}, {4.100, 90}, {4.111, 91}, {4.122, 92}, {4.132, 93}, {4.143, 94}, {4.153, 95}, {4.163, 96}, {4.173, 97}, {4.182, 98}, {4.191, 99}, {4.200, 100}} --## Table standard (Empirique & théorique)
//{{2.8, 0}, {2.942, 1}, {3.1, 2}, {3.258, 3}, {3.401, 4}, {3.485, 5}, {3.549, 6}, {3.601, 7}, {3.637, 8}, {3.664, 9}, {3.679, 10}, {3.683, 11}, {3.689, 12}, {3.692, 13}, {3.705, 14}, {3.71, 15}, {3.713, 16}, {3.715, 17}, {3.72, 18}, {3.731, 19}, {3.735, 20}, {3.744, 21}, {3.753, 22}, {3.756, 23}, {3.758, 24}, {3.762, 25}, {3.767, 26}, {3.774, 27}, {3.78, 28}, {3.783, 29}, {3.786, 30}, {3.789, 31}, {3.794, 32}, {3.797, 33}, {3.8, 34}, {3.802, 35}, {3.805, 36}, {3.808, 37}, {3.811, 38}, {3.815, 39}, {3.818, 40}, {3.822, 41}, {3.826, 42}, {3.829, 43}, {3.833, 44}, {3.837, 45}, {3.84, 46}, {3.844, 47}, {3.847, 48}, {3.85, 49}, {3.854, 50}, {3.857, 51}, {3.86, 52}, {3.863, 53}, {3.866, 54}, {3.87, 55}, {3.874, 56}, {3.879, 57}, {3.888, 58}, {3.893, 59}, {3.897, 60}, {3.902, 61}, {3.906, 62}, {3.911, 63}, {3.918, 64}, {3.923, 65}, {3.928, 66}, {3.939, 67}, {3.943, 68}, {3.949, 69}, {3.955, 70}, {3.961, 71}, {3.968, 72}, {3.974, 73}, {3.981, 74}, {3.987, 75}, {3.994, 76}, {4.001, 77}, {4.008, 78}, {4.014, 79}, {4.021, 80}, {4.029, 81}, {4.036, 82}, {4.044, 83}, {4.052, 84}, {4.062, 85}, {4.074, 86}, {4.085, 87}, {4.095, 88}, {4.105, 89}, {4.111, 90}, {4.116, 91}, {4.12, 92}, {4.125, 93}, {4.129, 94}, {4.135, 95}, {4.145, 96}, {4.176, 97}, {4.179, 98}, {4.193, 99}, {4.2, 100}}                 --## Table Robbe originale fiable (Départ à 2.8V
double myArrayPercentList[101] =                                                       
              {3, 3.093, 3.196, 3.301, 3.401, 3.477, 3.544, 3.601, 3.637, 3.664, 3.679, 3.683, 3.689, 3.692, 3.705, 3.71, 
			  3.713, 3.715, 3.72, 3.731, 3.735, 3.744, 3.753, 3.756, 3.758, 3.762, 3.767, 3.774, 3.78, 3.783, 3.786, 3.789, 
			  3.794, 3.797, 3.8, 3.802, 3.805, 3.808, 3.811, 3.815, 3.818, 3.822, 3.825, 3.829, 3.833, 3.836, 3.84, 3.843, 3.847, 3.85, 3.854, 
			  3.857, 3.86, 3.863, 3.866, 3.87, 3.874, 3.879, 3.888, 3.893, 3.897, 3.902, 3.906, 3.911, 3.918, 3.923, 3.928, 3.939, 3.943, 
			  3.949, 3.955, 3.961, 3.968, 3.974, 3.981, 3.987, 3.994, 4.001, 4.007, 4.014, 4.021, 4.029, 4.036, 4.044, 4.052, 4.062, 4.074, 4.085, 4.095, 
			  4.105, 4.111, 4.116, 4.12, 4.125, 4.129, 4.135, 4.145, 4.176, 4.179, 4.193, 4.2};   //## Table Robbe fiable modifiée pour départ à 3.0V

int8_t percentBat(double targetVoltage) {
  int8_t result = 0;
  
#ifdef IMU_DEBUG
       Serial.print("percentbat : ");
       Serial.println(targetVoltage);
#endif IMU_DEBUG
  
  if ((targetVoltage > cellfull) or (targetVoltage < cellempty)) {
    if  (targetVoltage > cellfull) {                                            //## trap for odd values not in array
      result = 100;
    }
    if  (targetVoltage < cellempty) {
      result = 0;
    }
  }
  else {
    for (int8_t i=0; i<101; i++) {                                 //## method of finding percent in my array provided by on4mh (Mike)
      if (myArrayPercentList[i] >= targetVoltage) {
        result =  i;
        break;
      }
    } //for
  } //if
  
#ifdef IMU_DEBUG
       Serial.print("result : ");
       Serial.println(result);
#endif IMU_DEBUG
  
 return result;
}

