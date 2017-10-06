/*

This sketch allows the users to manage the Tian power management.
Users can power up or power down the MIPS side from the MCU serial console.

After upload the sketch, open the Serial Terminal and press the send button.

COMMANDS
----------------------
h - help
D - power down MIPS
U - power up MIPS

created March 2016
	by andrea@arduino.org

 */

//#define MIPS_PIN 32  //PA28 PIN 32

#include <EnergySaving.h>

char var;
EnergySaving nrgSave;

void setup() {
	// put your setup code here, to run once:
	Serial.begin(115200);

	//pinMode(MIPS_PIN,OUTPUT);    //MIPS
	//digitalWrite(MIPS_PIN,HIGH);
	nrgSave.noLowPowerMode();

	while(!Serial.available());;
	menu('h');
}

void loop() {
	// put your main code here, to run repeatedly:
	while (Serial.available()){
		var = Serial.read();
		menu(var);
	}
}

void menu( char var){
		 switch (var) {
			case 'D':
				Serial.println("MIPS OFF");
				Serial.println();
				//digitalWrite(MIPS_PIN,LOW);
				nrgSave.maxLowPowerMode();
				break;
			case 'U':
				Serial.println("MIPS ON");
				Serial.println();
				//digitalWrite(MIPS_PIN,HIGH);
				nrgSave.noLowPowerMode();
				break;
			case 'h':
				Serial.println("COMMANDS");
				Serial.println("----------------------");
				Serial.println("h - help");
				Serial.println("D - power down MIPS");
				Serial.println("U - power up MIPS");
				Serial.println();
				break;
		}

}
