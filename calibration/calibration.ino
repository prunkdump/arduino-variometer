#include <Arduino.h>
#include <I2Cdev.h>
#include <ms5611.h>
#include <vertaccel.h>
#include <inv_mpu.h>
#include <avr/pgmspace.h>

#define CALIBRATION_FILTER_SIZE 5000
#define CALIBRATION_FILTER_SIZEF 5000.0
#define CALIBRATION_WAIT_DURATION 3000
#define CALIBRATION_BASE_RADIUS 1.0
#define CALIBRATION_BASE_RADIUS_DRIFT 0.01
#define CALIBRATION_BASE_RADIUS_STEP 0.001
#define CALIBRATION_OPTIMIZATION_PRECISION 0.000001
            
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
const char usage11[] PROGMEM  = "  The accelerometer calibration procedure";
const char usage12[] PROGMEM  = "  need five accel vectors from the five upper ";
const char usage13[] PROGMEM  = "  orientations. That is to say with the";
const char usage14[] PROGMEM  = "  accelerometer pointing :";
const char usage15[] PROGMEM  = "    -> toward the sky, flat on the groud";
const char usage16[] PROGMEM  = "    -> toward the left on it's left side";
const char usage17[] PROGMEM  = "    -> toward the right on it's right side";
const char usage18[] PROGMEM  = "    -> toward you on it's bottom side";
const char usage19[] PROGMEM  = "    -> back to you, on it's top side";
const char usage20[] PROGMEM  = "";
const char usage21[] PROGMEM  = "  For each measure :";
const char usage22[] PROGMEM  = "  1) Make sure that the accel vector is";
const char usage23[] PROGMEM  = "     stabilized with multiple 'd' command.";
const char usage24[] PROGMEM  = "  2) Record the accel vector with the ";
const char usage25[] PROGMEM  = "     'c' command.";
const char usage26[] PROGMEM  = "";
const char usage27[] PROGMEM  = "  Once the calibration's done. You can check";
const char usage28[] PROGMEM  = "  the result by pointing the accelerometer";
const char usage29[] PROGMEM  = "  in different direction with the 'c' ";
const char usage30[] PROGMEM  = "  command. The results must be as close as";
const char usage31[] PROGMEM  = "  possible to 1.000. If not press 'r' and restart.";
const char usage32[] PROGMEM  = "-------------------------";
const char usage33[] PROGMEM  = "";

const char* const usage[] PROGMEM = { usage01, usage02, usage03, usage04, usage05,
                                      usage06, usage07, usage08, usage09, usage10,
                                      usage11, usage12, usage13, usage14, usage15,
                                      usage16, usage17, usage18, usage19, usage20,
                                      usage21, usage22, usage23, usage24, usage25,
                                      usage26, usage27, usage28, usage29, usage30,
                                      usage31, usage32, usage33 };
#define USAGE_TEXT_LINE_COUNT 33
                                      
const char calStepMsg01[] PROGMEM  = "1) Calibrate with the accelerometer flat on the ground";
const char calStepMsg02[] PROGMEM  = "2) Calibrate with the accelerometer on it's left side";
const char calStepMsg03[] PROGMEM  = "3) Calibrate with the accelerometer on it's right side";
const char calStepMsg04[] PROGMEM  = "4) Calibrate with the accelerometer on it's bottom side";
const char calStepMsg05[] PROGMEM  = "5) Calibrate with the accelerometer on it's top side";

const char* const calStepMsg[] PROGMEM = { calStepMsg01, calStepMsg02, calStepMsg03, calStepMsg04, calStepMsg05};

const char resetMsg[] PROGMEM  = "Reset calibration !";
const char waitMsg[] PROGMEM  = "Don't move the accelerometer and wait...";
const char recordMsg[] PROGMEM  = "Starting measure...";
const char optimizationMsg[] PROGMEM  = "Starting optimization...";

const char calMsgB01[] PROGMEM  = "----------------";
const char calMsgB02[] PROGMEM  = "CALIBRATION DONE ";
const char calMsgB03[] PROGMEM  = "----------------";
const char calMsgB04[] PROGMEM  = "";
const char calMsgB05[] PROGMEM  = "Copy the following lines to libraries/vertaccel/vertaccel.h";
const char calMsgB06[] PROGMEM  = "";

const char* const calMsgB[] PROGMEM = {calMsgB01, calMsgB02, calMsgB03, calMsgB04, calMsgB05, calMsgB06};
#define CALB_TEXT_LINE_COUNT 6

const char calMsgE01[] PROGMEM  = "";
const char calMsgE02[] PROGMEM  = "Run multiple 'c' command to check the result.";
const char calMsgE03[] PROGMEM  = "----------------";

const char* const calMsgE[] PROGMEM = {calMsgE01, calMsgE02, calMsgE03};
#define CALE_TEXT_LINE_COUNT 3


/*********************/               
/* display functions */
/*********************/
char stringBuffer[65];

void displayText( const char* const* text, int textLinesCount) {
  for(int i = 0; i<textLinesCount; i++) {
    strcpy_P(stringBuffer, (char*)pgm_read_word(&(text[i])));
    Serial.println(stringBuffer);
  }
}

void displayString(const char* string) {
  strcpy_P(stringBuffer, string);
  Serial.println(stringBuffer);
}



/*************************/
/* calibration variables */
/*************************/

/* accel vectors */
double v[5*3];

/* calibration status */
boolean calibrated;
int calStep;

/* mean filter result */
double maccel[3];
double mup[3];
double mva;

/* center of the accel vectors */
double calibrationCenter[3];
double calibrationRadius;
               


/*-----------------*/
/*      SETUP      */
/*-----------------*/
void setup() {
   
  /***********************/
  /* init accelerometer */
  /**********************/
  Fastwire::setup(400,0);
  vertaccel_init(false);
  
  /***************/
  /* init serial */
  /***************/
  Serial.begin(9600);
  
  /*********************/
  /* display procedure */
  /*********************/
  displayText(usage, USAGE_TEXT_LINE_COUNT);
  
  calibrated = false;
  calStep = 0;
  displayString( (char*)pgm_read_word(&(calStepMsg[calStep])) );
  
}


/*************************/
/* calibration functions */
/*************************/

/* given three vector and a radius, find the sphere's center */
void computeCenter(double* v1, double* v2, double* v3, double radius, double* center) {
            
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
  q[2]= (v1[0]-eq1[3])*(v1[0]-eq1[3]) + v1[1]*v1[1] + (v1[2]-eq2[3])*(v1[2]-eq2[3])-radius;
      
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
}

/* given 5 vectors and a sphere center, compute distance from center variance */
double computeDistanceVariance(double *v, double* center) {

  /* compute distances */
  double pointDistance[5];
  
  for( int i = 0; i<5; i++ ) {
    double* val = &v[i*3];
    pointDistance[i] = sqrt( (val[0]-center[0])*(val[0]-center[0]) + (val[1]-center[1])*(val[1]-center[1]) + (val[2]-center[2])*(val[2]-center[2]) );
  }
  
  /* compute mean */
  double mean = 0.0;
  
  for( int i = 0; i<5; i++ ) {
    mean += pointDistance[i];
  }
  
  mean /= 5.0;
  
  /* compute var */
  double var = 0.0;
  
  for( int i = 0; i<5; i++ ) {
    var += (pointDistance[i] - mean)*(pointDistance[i] - mean);
  }
  
  var /= 5.0;
  
  return var;
}


/*----------------*/
/*      LOOP      */
/*----------------*/
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
      displayString(resetMsg);
      displayString( (char*)pgm_read_word(&(calStepMsg[calStep])) );
    } else if( c == 'd' || c == 'c' ) {
      
      /**************/
      /* start read */
      /**************/
      
      /* stabilize the accelerometer */
      displayString(waitMsg);
      unsigned long currentTime = millis();
      while( millis() - currentTime < CALIBRATION_WAIT_DURATION ) {
        vertaccel_rawReady(accel, upVector, &va);
      }
      
      /* starting measures with mean filter */
      displayString(recordMsg);
      
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
      
      /* if needed diaply result */
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
          if( calStep < 5 ) {
            //double cv[3];
            double* cv = &(v[3*calStep]);
            cv[0] = maccel[0];
            cv[1] = maccel[1];
            cv[2] = maccel[2];
            calStep++;
            Serial.print("Recorded point number ");
            Serial.print(calStep, DEC);
            Serial.print(" ! \n");
            if(calStep < 5) {
              displayString( (char*)pgm_read_word(&(calStepMsg[calStep])) );
            }
          }
        
          /* if we have the 5 points */
          if( calStep == 5 ) {
            
            /****************************/
            /* make radius optimization */
            /****************************/
            displayString(optimizationMsg);
            double baseRadius = CALIBRATION_BASE_RADIUS;
            double baseRadiusDrift = CALIBRATION_BASE_RADIUS_DRIFT;
            double baseStep = CALIBRATION_BASE_RADIUS_STEP;
            double bestDistance = 100000.0;
            double bestRadius;
                        
            while( baseStep > CALIBRATION_OPTIMIZATION_PRECISION ) {
              
              double currentRadius = baseRadius - baseRadiusDrift;
              double currentDistance;
              
              while( currentRadius <  baseRadius + baseRadiusDrift ) {
                computeCenter(&v[0], &v[3], &v[9], currentRadius, calibrationCenter); //flat, left, bottom
                currentDistance = computeDistanceVariance(v, calibrationCenter);
                if( currentDistance <  bestDistance ) {
                  bestDistance = currentDistance;
                  bestRadius = currentRadius;
                }
                currentRadius += baseStep;
              }
              
              baseRadius = bestRadius;
              baseStep /= 10.0;
              baseRadiusDrift /= 10.0;
            }
            
            
            computeCenter(&v[0], &v[3], &v[9], bestRadius, calibrationCenter); 
            Serial.print("Optimized radius : ");
            Serial.println(bestRadius, 6);
         
            /* display result */
            displayText(calMsgB, CALB_TEXT_LINE_COUNT);
            Serial.print("#define VERTACCEL_ACCEL_CAL_X (");
            Serial.print(-calibrationCenter[0], 5);
            Serial.print(")\n");
            Serial.print("#define VERTACCEL_ACCEL_CAL_Y (");
            Serial.print(-calibrationCenter[1], 5);
            Serial.print(")\n");
            Serial.print("#define VERTACCEL_ACCEL_CAL_Z (");
            Serial.print(-calibrationCenter[2], 5);
            Serial.print(")\n");
            displayText(calMsgE, CALE_TEXT_LINE_COUNT);
            calibrated = true;
          }
        
        } else {
        
          /************************/
          /* distance computation */
          /************************/
          double dist = sqrt((maccel[0]-calibrationCenter[0])*(maccel[0]-calibrationCenter[0]) + (maccel[1]-calibrationCenter[1])*(maccel[1]-calibrationCenter[1]) + (maccel[2]-calibrationCenter[2])*(maccel[2]-calibrationCenter[2]));
          Serial.print("calibrated distance : ");
          Serial.print(dist, 5);
          Serial.print("\n");
        }
      }
    }
  }
}
        
        
        
      
      
 
