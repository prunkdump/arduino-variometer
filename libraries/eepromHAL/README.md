# TONEHAL Library

ToneHAL is an arduino abstraction library for sound management 

Multi-protocol :
									PWM		1 pin
									PWM		2 pins
									DAC
									I2S
									
Multi-platform :
									Arduino Pro-Mini
									Atmel SAM D21 - MKZERO 
									Espressif ESP32
									
## Setting

	in toneHAL.h

									TONEI2S		I2S interface
									TONEDAC		DAC interface
									TONEAC		2 pins Push-Pull PWM
									TONE 			1 pin PWM
									
  ProMini :                                                                      
                PWM 1 pin                         OK                             
                PWM AC 2 pins                     OK                             
                DAC                               Not available                  
                I2S                               Not available                  
                                                                                 
  MKZERO                                                                         
                PWM 1 pin                         OK                             
                PWM 2 pins                        OK                             
                DAC                               OK                             
                I2S                               not yet developed              
                                                                                 
 ESP32          PWM 1 pin                         OK                             
                PWM 2 pins                        not yet developed              
                DAC                               in development                 
                I2S                               not yet developed              
								
##	VERSION 1.4.1
								