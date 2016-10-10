#include <Arduino.h>
#include <I2Cdev.h>
#include <ms5611.h>
#include <vertaccel.h>
#include <inv_mpu.h>
#include <avr/pgmspace.h>

#define CALIBRATION_FILTER_SIZE 1000
#define CALIBRATION_FILTER_SIZEF 1000.0

const char usage01[] PROGMEM  = "-------------------------";
const char usage02[] PROGMEM  = "ACCELEROMETER CALIBRATION";
const char usage03[] PROGMEM  = "-------------------------";
const char usage04[] PROGMEM  = "";
const char usage05[] PROGMEM  = "Commands :";
const char usage06[] PROGMEM  = "  d : display current accel vector";
const char usage07[] PROGMEM  = "  c : make calibration (record accel)";
const char usage08[] PROGMEM  = "  r : restart calibration to the beginning";
const char usage09[] PROGMEM  = "";
const char usage10[] PROGMEM  = "Procedure :";
const char usage11[] PROGMEM  = "  The accelerometer calibration need three";
const char usage12[] PROGMEM  = "  accel vectors from three orientations.";
const char usage13[] PROGMEM  = "  The orientations must not be opposite.";
const char usage14[] PROGMEM  = "  For example take three measures with the";
const char usage15[] PROGMEM  = "  accelerometer pointing :";
const char usage16[] PROGMEM  = "    -> toward the sky on it's botton side";
const char usage17[] PROGMEM  = "    -> toward the left on it's left side";
const char usage18[] PROGMEM  = "    -> toward you on it's bottom side";
const char usage19[] PROGMEM  = "";
const char usage20[] PROGMEM  = "  For each measure :";
const char usage21[] PROGMEM  = "  1) Make sure that the accel vector is";
const char usage22[] PROGMEM  = "     stabilized with multiple 'd' command.";
const char usage23[] PROGMEM  = "  2) Record the accel vector with the ";
const char usage24[] PROGMEM  = "     'c' command.";
const char usage25[] PROGMEM  = "";
const char usage26[] PROGMEM  = "  Once the calibration done. You can check";
const char usage27[] PROGMEM  = "  the result by pointing the accelerometer";
const char usage28[] PROGMEM  = "  in different direction with the 'c' ";
const char usage29[] PROGMEM  = "  command. The results must be as close as";
const char usage30[] PROGMEM  = "  possible to 1.000.";
const char usage31[] PROGMEM  = "-------------------------";

const char* const usage[] PROGMEM = { usage01, usage02, usage03, usage04, usage05,
                                      usage06, usage07, usage08, usage09, usage10,
                                      usage11, usage12, usage13, usage14, usage15,
                                      usage16, usage17, usage18, usage19, usage20,
                                      usage21, usage22, usage23, usage24, usage25,
                                      usage26, usage27, usage28, usage29, usage30, usage31 };
                                      
char stringBuffer[50];

/*-----------------*/
/*      SETUP      */
/*-----------------*/
void setup() {
   
  /***********************/
  /* init accelerometer */
  /**********************/
  Fastwire::setup(400,0);
  vertaccel_init();
  
  /***************/
  /* init serial */
  /***************/
  Serial.begin(9600);
  
  /********************/
  /* display commands */
  /********************/
  for(int i = 0; i<31; i++) {
    strcpy_P(stringBuffer, (char*)pgm_read_word(&(usage[i])));
    Serial.println(stringBuffer);
  }
    
  
}

/*----------------*/
/*      LOOP      */
/*----------------*/

/* calibration point */
double v[3*3];
boolean calibrated = false;
int calStep = 0;
double maccel[3];
double mup[3];
double mva;
double center[3];

void loop() {
  /********************************/
  /* flush the accelerometer FIFO */
  /********************************/
  double accel[3];
  double upVector[3];
  double va;
  vertaccel_rawReady(accel, upVector, &va);
  
  /******************/
  /* check commands */
  /******************/
  if( Serial.available() ) {
    uint8_t c;
    while( Serial.available() ) {
      c = Serial.read();
    }
    if( c == 'r' ) {
      calibrated = false;
      calStep = 0;
      Serial.print("Reset calibration !\n");
    } else if( c == 'd' || c == 'c' ) {
      
      /**************/
      /* start read */
      /**************/
      
      /* stabilize the accelerometer */
      Serial.print("Don't move the accelerometer and wait...\n");
      unsigned long currentTime = millis();
      while( millis() - currentTime < 5000 ) {
        vertaccel_rawReady(accel, upVector, &va);
      }
      
      /* starting measures */
      Serial.print("Starting measure...\n");
      
      int count = 0;
      
      maccel[0] = 0.0;
      maccel[1] = 0.0;
      maccel[2] = 0.0;
      
      mup[0] = 0.0;
      mup[1] = 0.0;
      mup[2] = 0.0;
      
      mva = 0;
  
      while( count < CALIBRATION_FILTER_SIZE ) {
        if( vertaccel_rawReady(accel, upVector, &va) ) {
          maccel[0] += accel[0];
          maccel[1] += accel[1];
          maccel[2] += accel[2];
          
          mup[0] += upVector[0];
          mup[1] += upVector[1];
          mup[2] += upVector[2];
          
          mva += va;
          
          count++;
        }
      }
      
      maccel[0] /= (double)count;
      maccel[1] /= (double)count;
      maccel[2] /= (double)count;
      mup[0] /= (double)count;
      mup[1] /= (double)count;
      mup[2] /= (double)count;
      mva /= (double)count;
      
      if( c == 'd' ) {
        Serial.print("accel=(");
        Serial.print(maccel[0], 5);
        Serial.print(", ");
        Serial.print(maccel[1], 5);
        Serial.print(", ");
        Serial.print(maccel[2], 5);
        Serial.print(")\n");
      } else if( c == 'c' ) {
      
        /***********************/
        /* calibrate if needed */
        /***********************/
        if(!calibrated) {
        
          /* if we not have the 3 points yet */
          if( calStep < 3 ) {
            //double cv[3];
            double* cv = &(v[3*calStep]);
            cv[0] = maccel[0];
            cv[1] = maccel[1];
            cv[2] = maccel[2];
            calStep++;
            Serial.print("Recorded point number ");
            Serial.print(calStep, DEC);
            Serial.print(" ! \n");
          }
        
          /* if we have the 3 points */
          if( calStep == 3 ) {
          
            /***************/
            /* calibration */
            /***************/
            double* v1 = &v[0];
            double* v2 = &v[3];
            double* v3 = &v[6];
          
            /* compute midppoints */
            double m1[3];
            double m2[3];
      
            m1[0] = (v1[0]+v2[0])/2.0;
            m1[1] = (v1[1]+v2[1])/2.0;
            m1[2] = (v1[2]+v2[2])/2.0;
            m2[0] = (v1[0]+v3[0])/2.0;
            m2[1] = (v1[1]+v3[1])/2.0;
            m2[2] = (v1[2]+v3[2])/2.0;
      
            /* compute plane equations */
            double eq1[4];
            double eq2[4];
      
            eq1[0] = v2[0] - v1[0];
            eq1[1] = v2[1] - v1[1];
            eq1[2] = v2[2] - v1[2];
            eq1[3] = eq1[0]*m1[0] + eq1[1]*m1[1] + eq1[2]*m1[2];
      
            eq2[0] = v3[0] - v1[0];
            eq2[1] = v3[1] - v1[1];
            eq2[2] = v3[2] - v1[2];
            eq2[3] = eq2[0]*m2[0] + eq2[1]*m2[1] + eq2[2]*m2[2];
 
            /* simplify equ2 */
            double fac = eq2[0]/eq1[0];
            eq2[0] -= eq1[0]*fac;
            eq2[1] -= eq1[1]*fac;
            eq2[2] -= eq1[2]*fac;
            eq2[3] -= eq1[3]*fac;
            eq2[0] /= eq2[2]; 
            eq2[1] /= eq2[2];
            eq2[3] /= eq2[2];
            eq2[2] /= eq2[2];
      
            /* simplify equ1 */
            fac = eq1[2];
            eq1[1] -= eq2[1]*fac;
            eq1[2] -= eq2[2]*fac;
            eq1[3] -= eq2[3]*fac;
            eq1[1] /= eq1[0]; 
            eq1[2] /= eq1[0];
            eq1[3] /= eq1[0];
            eq1[0] /= eq1[0];
 
            /* get quadratic equation */
            double q[3];
            q[0]= eq1[1]*eq1[1] + 1 + eq2[1]*eq2[1];
            q[1]= 2*eq1[1]*(v1[0]-eq1[3])-2*v1[1]+2*eq2[1]*(v1[2]-eq2[3]);
            q[2]= (v1[0]-eq1[3])*(v1[0]-eq1[3]) + v1[1]*v1[1] + (v1[2]-eq2[3])*(v1[2]-eq2[3])-1;
      
            /* solve quadratic */
            double d = q[1]*q[1] - 4*q[0]*q[2];
            double y1 = (-q[1]-sqrt(d))/(2*q[0]);
            double y2 = (-q[1]+sqrt(d))/(2*q[0]);
      
            /* compute points */
            if( -0.1 < y1 && y1 < 0.1 ) {      
              center[1] = y1;
              center[0] = eq1[3]-eq1[1]*y1;
              center[2] = eq2[3]-eq2[1]*y1;
            } else {
              center[1] = y2;
              center[0] = eq1[3]-eq1[1]*y2;
              center[2] = eq2[3]-eq2[1]*y2;
            }
         
            /* display result */
            Serial.print("----------------\n");
            Serial.print("CALIBRATION DONE \n");
            Serial.print("----------------\n");
            Serial.print("\n");
            Serial.print("Copy the following lines to libraries/vertaccel/vertaccel.h\n");
            Serial.print("\n");
            Serial.print("#define VERTACCEL_ACCEL_CAL_X (");
            Serial.print(-center[0], 5);
            Serial.print(")\n");
            Serial.print("#define VERTACCEL_ACCEL_CAL_Y (");
            Serial.print(-center[1], 5);
            Serial.print(")\n");
            Serial.print("#define VERTACCEL_ACCEL_CAL_Z (");
            Serial.print(-center[2], 5);
            Serial.print(")\n");
            Serial.print("\n");
            Serial.print("Run multiple 'c' command to check the result. \n");
            Serial.print("----------------\n");
            calibrated = true;
          }
        
        } else {
        
          /************************/
          /* distance computation */
          /************************/
          double dist = sqrt((maccel[0]-center[0])*(maccel[0]-center[0]) + (maccel[1]-center[1])*(maccel[1]-center[1]) + (maccel[2]-center[2])*(maccel[2]-center[2]));
          Serial.print("calibrated distance : ");
          Serial.print(dist, 5);
          Serial.print("\n");
        }
      }
    }
  }
}
        
        
        
      
      
 
