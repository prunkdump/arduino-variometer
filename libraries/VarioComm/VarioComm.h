#ifndef VARIOCOMM_H
#define VARIOCOMM_H

#include <Arduino.h>

/************************************************************/
/* VARIOCOMM                                                */ 
/* -> Communication PC->Arduino via USB                     */
/************************************************************/

class VarioComm {

 public:
  VarioComm() {};
  void setup();
  void Execute();
  void printCommandHeader(String header);
  void checkConnection(byte value);
  void set_USB_Mode (byte value);
  void resetALL();
  void readFile(String fileName);
  void writeFile(String fileName, String data);
  void appendFile(String fileName, String data);
  void setFileName(String fileName);
  void diskConnectionStatus();
  void USBdiskMount();
  void fileOpen();
  boolean setByteRead(byte numBytes);
  int getFileSize();
  void fileRead();
  void fileWrite(String data);
  boolean continueRead();
  boolean fileCreate();
  void fileDelete(String fileName);
  void filePointer(boolean fileBeginning);
  void fileClose(byte closeCmd);
  boolean waitForResponse(String errorMsg);
  boolean waitForConnection();
  byte getResponseFromUSB();
  void blinkLED();


 private:
   byte computerByte;           //used to store data coming from the computer
   byte USB_Byte;               //used to store data coming from the USB stick
   int LED = 13;                //the LED is connected to digital pin 13 
   int timeOut = 2000;          //TimeOut is 2 seconds. This is the amount of time you wish to wait for a response from the CH376S module.
   String wrData = "What is the meaning of life ?";     //We will write this data to a newly created file.
   String wrData2 = "42";                                   //We will append this data to an already existing file.

};

#endif
