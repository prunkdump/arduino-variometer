#ifndef VARIOCOMM_H
#define VARIOCOMM_H

#include <Arduino.h>
#include <CmdMessenger.h>  // CmdMessenger
#include <SPI.h>
#include <SD.h>
#include <VarioSettings.h>


void attachCommandCallbacks();
void OnUnknownCommand();
void OnCommandList();
void OnNbFile();
void OnListFile();
void OnDownloadFile();
void OnDownload();
void OnExit();
void DownloadFile(String FileName);
void ShowCommands(); 
void initVarioComm(); 
void usbConnectedMode(void); 
void VerifySDCard(void);
 
extern byte statePowerInt;

#endif
