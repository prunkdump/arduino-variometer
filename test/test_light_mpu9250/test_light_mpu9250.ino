#include <LightInvensense.h>
#include <I2Cdev.h>
#include <ms5611_zero.h>
#include <VarioSettings.h>


unsigned long t;

void setup() {

  Serial.begin(9600);
  while(!Serial) { }
  Serial.println("Start !"); 

  pinMode(VARIO_PIN_ALIM, OUTPUT);
  digitalWrite(VARIO_PIN_ALIM, HIGH);   // turn on power cards )

  /* init device */
  //Fastwire::setup(FASTWIRE_SPEED, 0);

  delay(1000);
  
  Wire.begin();
  Wire.setClock(400000); // set clock frequency AFTER Wire.begin()

  if( fastMPUInit() < 0 ) {
    Serial.println("Failed to init device !"); 
  }

  t = millis();
}

short gyro[3];
short accel[3];
long quat[4];
boolean newData = false;

void loop() {

  /* read fifo */
 // fastMPUReadFIFO(gyro, accel, quat);

  newData = false;
  /* display */
 // if( millis() - t > 1000 ) {
  while( fastMPUReadFIFO(gyro, accel, quat) >= 0 ) {
    newData = true;
  }

  /* if new data compute vertical acceleration */
  if( newData ) {

    t = millis();
    Serial.print( accel[0], DEC);
    Serial.print(", ");
    Serial.print( accel[1], DEC);
    Serial.print(", ");
    Serial.println( accel[2], DEC);
    Serial.flush();
  }  

}
