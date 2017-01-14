#include <Arduino.h>
#include <I2Cdev.h>
#include <ms5611.h>
#include <vertaccel.h>
#include <EEPROM.h>
#include <inv_mpu.h>
#include <avr/pgmspace.h>
#include <accelcalibrator.h>

AccelCalibrator calibrator;
            
const char usage01[] PROGMEM  = "-------------------------";
const char usage02[] PROGMEM  = "ACCELEROMETER CALIBRATION";
const char usage03[] PROGMEM  = "-------------------------";
const char usage04[] PROGMEM  = "";
const char usage05[] PROGMEM  = "Commands :";
const char usage06[] PROGMEM  = "  d : display current accel vector";
const char usage07[] PROGMEM  = "  m : measure accel vertor for calibration";
const char usage08[] PROGMEM  = "  c : start calibration";
const char usage09[] PROGMEM  = "  r : restart calibration to the beginning";
const char usage10[] PROGMEM  = "";
const char usage11[] PROGMEM  = "Procedure :";
const char usage12[] PROGMEM  = "  The accelerometer calibration procedure";
const char usage13[] PROGMEM  = "  need five accel vectors from the five upper ";
const char usage14[] PROGMEM  = "  orientations. That is to say with the";
const char usage15[] PROGMEM  = "  accelerometer pointing :";
const char usage16[] PROGMEM  = "    -> toward the sky, flat on the groud";
const char usage17[] PROGMEM  = "    -> toward the left on it's left side";
const char usage18[] PROGMEM  = "    -> toward the right on it's right side";
const char usage19[] PROGMEM  = "    -> toward you on it's bottom side";
const char usage20[] PROGMEM  = "    -> back to you, on it's top side";
const char usage21[] PROGMEM  = "";
const char usage22[] PROGMEM  = "  For each measure :";
const char usage23[] PROGMEM  = "  1) You can check the current accel vector";
const char usage24[] PROGMEM  = "     with multiple 'd' command.";
const char usage25[] PROGMEM  = "  2) Record the accel vector with the 'm' command";
const char usage26[] PROGMEM  = "     This can be done multiple times for each orientations";
const char usage27[] PROGMEM  = "";
const char usage28[] PROGMEM  = "  Once the five orientations recorded. You can";
const char usage29[] PROGMEM  = "  launch the calibration procedure with the 'c'";
const char usage30[] PROGMEM  = "  command. Use the 'd' command  to check the result.";
const char usage31[] PROGMEM  = "  The distance must be as close as possible to 1.000.";
const char usage32[] PROGMEM  = "  If you are not satisfied by the result. Make more";
const char usage33[] PROGMEM  = "  measures or reset teh calibration with the 'r' command.";
const char usage34[] PROGMEM  = "-------------------------";
const char usage35[] PROGMEM  = "";

const char* const usage[] PROGMEM = { usage01, usage02, usage03, usage04, usage05,
                                      usage06, usage07, usage08, usage09, usage10,
                                      usage11, usage12, usage13, usage14, usage15,
                                      usage16, usage17, usage18, usage19, usage20,
                                      usage21, usage22, usage23, usage24, usage25,
                                      usage26, usage27, usage28, usage29, usage30,
                                      usage31, usage32, usage33, usage34, usage35};
#define USAGE_TEXT_LINE_COUNT 33


const char separatorMsg[] PROGMEM  = "-------------------------";
const char measureReadyMsg[] PROGMEM  = "Ready to make measure.";
const char measureAndCalibrateReadyMsg[] PROGMEM  = "Ready to make measure or calibration.";
const char calibrationNotReadyMsg[] PROGMEM  = "Not enough measures to calibrate !";

const char measureValidMsg[] PROGMEM  = "GOOD measure, recorded for calibration";
const char measureInvalidMsg01[] PROGMEM  = "BAD measure, not recorded, possible problems are :";
const char measureInvalidMsg02[] PROGMEM  = "-> ambiguous orientation";
const char measureInvalidMsg03[] PROGMEM  = "-> orientation already done with lower standard deviation";
const char* const measureInvalidMsg[] PROGMEM = {measureInvalidMsg01, measureInvalidMsg02, measureInvalidMsg03};
#define MEASURE_INVALID_LINE_COUNT 3

const char resetMsg[] PROGMEM  = "Reset calibration !";
const char waitMsg[] PROGMEM  = "Don't move the accelerometer and wait...";
const char recordMsg[] PROGMEM  = "Starting measure...";


const char calMsgB01[] PROGMEM  = "-------------------------";
const char calMsgB02[] PROGMEM  = "CALIBRATION DONE ";
const char calMsgB03[] PROGMEM  = "-------------------------";
const char calMsgB04[] PROGMEM  = "";
const char calMsgB05[] PROGMEM  = "Here the new calibration coefficients :";
const char calMsgB06[] PROGMEM  = "";
const char* const calMsgB[] PROGMEM = {calMsgB01, calMsgB02, calMsgB03, calMsgB04, calMsgB05, calMsgB06};
#define CALB_TEXT_LINE_COUNT 6

const char calMsgE01[] PROGMEM  = "";
const char calMsgE02[] PROGMEM  = "Run multiple 'd' command to check the result.";
const char calMsgE03[] PROGMEM  = "-------------------------";
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


/*-----------------*/
/*      SETUP      */
/*-----------------*/
void setup() {
   
  /*************************************/
  /* init accelerometer and calibrator */
  /*************************************/
  Fastwire::setup(400,0);
  calibrator.init();
  
  /***************/
  /* init serial */
  /***************/
  Serial.begin(9600);
  
  /*********************/
  /* display procedure */
  /*********************/
  displayText(usage, USAGE_TEXT_LINE_COUNT);
  
  /* display current calibration coeffs */
  Serial.print("Current calibration coefficients : \nx : ");
  Serial.print(calibrator.calibration[0], 5);
  Serial.print("\ny : ");
  Serial.print(calibrator.calibration[1], 5);
  Serial.print("\nz : ");
  Serial.print(calibrator.calibration[2], 5);
  Serial.print("\n\n");
       
  
  displayString(measureReadyMsg);
  
}

/*----------------*/
/*      LOOP      */
/*----------------*/
void loop() {

  /******************/
  /* check commands */
  /******************/
  if( Serial.available() ) {
    uint8_t c;
    while( Serial.available() ) {
      c = Serial.read();
    }
    if( c == 'r' ) {
      /*********/
      /* reset */
      /*********/
      calibrator.reset();
      displayString(resetMsg);

    } else if( c == 'd' ) {
      /*******************/
      /* display measure */
      /*******************/
      
      /* make measure */
      displayString(waitMsg);
      displayString(recordMsg);
      if( ! calibrator.calibrated ) {
        calibrator.measure();
      } else {
        calibrator.calibratedMeasure();
      }
      
      /* display the measure */
      Serial.print("accel=(");
      Serial.print(calibrator.measuredAccel[0], 5);
      Serial.print(", ");
      Serial.print(calibrator.measuredAccel[1], 5);
      Serial.print(", ");
      Serial.print(calibrator.measuredAccel[2], 5);
      Serial.print(")\n");
      Serial.print("Standard deviation = ");
      Serial.print(calibrator.measuredAccelSD, 5);
      Serial.print("\n");
      
      /* if calibrated display the distance */
      if( calibrator.calibrated ) {
        double dist = sqrt(calibrator.measuredAccel[0]*calibrator.measuredAccel[0] + calibrator.measuredAccel[1]*calibrator.measuredAccel[1] + calibrator.measuredAccel[2]*calibrator.measuredAccel[2]);
        Serial.print("calibrated distance : ");
        Serial.print(dist, 5);
        Serial.print("\n");
      }
    } else if( c == 'm' ) {
      
      /******************/
      /* record measure */
      /******************/
      
      /* make measure */
      displayString(waitMsg);
      displayString(recordMsg);
      calibrator.measure();
      
      /* display the measure */
      Serial.print("accel=(");
      Serial.print(calibrator.measuredAccel[0], 5);
      Serial.print(", ");
      Serial.print(calibrator.measuredAccel[1], 5);
      Serial.print(", ");
      Serial.print(calibrator.measuredAccel[2], 5);
      Serial.print(")\n");
      Serial.print("Standard deviation = ");
      Serial.print(calibrator.measuredAccelSD, 5);
      Serial.print("\n");
      
      /* record the result */
      boolean measureValid = calibrator.pushMeasure();
      if( measureValid ) {
        displayString(measureValidMsg);
      } else {
        displayText(measureInvalidMsg, MEASURE_INVALID_LINE_COUNT);
      }
    
    } else if( c == 'c' ) {
      
      /*************/
      /* calibrate */
      /*************/
      if( !calibrator.canCalibrate() ) {
        displayString(calibrationNotReadyMsg);
      } else {
        
        /* calibrate */
        calibrator.calibrate();
         
        /* display result */
        displayText(calMsgB, CALB_TEXT_LINE_COUNT);
        Serial.print("x : ");
        Serial.print(calibrator.calibration[0], 5);
        Serial.print("\ny : ");
        Serial.print(calibrator.calibration[1], 5);
        Serial.print("\nz : ");
        Serial.print(calibrator.calibration[2], 5);
        Serial.print("\n");
        displayText(calMsgE, CALE_TEXT_LINE_COUNT);
      }
    }
    
    /* next */
    displayString(separatorMsg);
    if( ! calibrator.canCalibrate() ) {
      displayString(measureReadyMsg);
    } else {
      displayString(measureAndCalibrateReadyMsg);
    }
  }
}

