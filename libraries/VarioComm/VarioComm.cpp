//   Available commands
//   0;                  - This command list
//   1;                  - Init + Nb File
//   2;                  - List File
//   3,<nb file>;        - Download nb file
//   4;                  - download files
//  


#include <CmdMessenger.h>  // CmdMessenger
#include <SPI.h>
#include <SD.h>
#include <VarioSettings.h>
#include <VarioComm.h>

// This is the list of recognized commands.  
// In order to receive, attach a callback function to these events
enum
{
  kCommandList         , // Command to request list of available commands
  kNbFile              , // Command to request number of file 
  kListFile            , // Command to request number of file 
  kDownloadFile        , // Command to request download file IGC 
  kDownload            , // Command to request download
  kExit                , // Command to request exit
};

 String TabFile[20];
 int compteurFile = 0;
 bool VarioCommConnect = false;


// Attach a new CmdMessenger object to the default Serial port
CmdMessenger cmdMessengerUsb = CmdMessenger(Serial);

// Callbacks define on which received commands we take action
void attachCommandCallbacks()
{
  // Attach callback methods
  cmdMessengerUsb.attach(OnUnknownCommand);
  cmdMessengerUsb.attach(kCommandList, OnCommandList);
  cmdMessengerUsb.attach(kNbFile, OnNbFile);
  cmdMessengerUsb.attach(kListFile, OnListFile);
  cmdMessengerUsb.attach(kDownloadFile, OnDownloadFile);
  cmdMessengerUsb.attach(kDownload, OnDownload);
  cmdMessengerUsb.attach(kExit, OnExit);
}

// Called when a received command has no attached function
void OnUnknownCommand()
{
#ifdef PROG_DEBUG 
  Serial.println("This command is unknown!");
#endif
  Serial.println("#ERROR#");
  ShowCommands();
}

// Callback function that shows a list of commands
void OnCommandList()
{
  ShowCommands();
}

// Callback function that sets led on or off
void OnNbFile()
{

  if (VarioCommConnect == false) {
	 VerifySDCard(); 
  }

  Serial.println("#NB FILE#");
  Serial.println(compteurFile);    
}

void VerifySDCard(void) {
  File root;

  Serial.println("#GNUVARIO CONNECTED#");

  VarioCommConnect = true;
  
  root = SD.open("/");

  while (true) {

    File entry =  root.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
#ifdef PROG_DEBUG 
    Serial.println(entry.name());
#endif
    if (! entry.isDirectory()) {
      // files have sizes, directories do not
      String stringTmp = entry.name(); 
      int bodyTag = stringTmp.indexOf(".IGC");
      if (bodyTag > 0) {
        TabFile[compteurFile] = stringTmp;
        compteurFile++;
      }
    }
    entry.close();
  }	
}

void OnListFile()
{
  if (VarioCommConnect == false) {
	 VerifySDCard(); 
  }
	
  Serial.println("#LIST FILE#");

  for(int i=0;i<compteurFile;i++) {
     Serial.println(TabFile[i]);
  }
  Serial.println("#END FILE LIST#");
}

// Callback function that download file
void OnDownloadFile()
{
  // Read file number argument, expects value between 1 to max file
  int NumFile = cmdMessengerUsb.readInt16Arg();  
  if (NumFile > compteurFile) {
#ifdef PROG_DEBUG 
    Serial.println("Number File Error");
#endif //PROG_DEBUG
  }
  else {
     Serial.println("#FILE NAME#");
     Serial.println(TabFile[NumFile-1]);
     Serial.println("#BEGIN TRANFERT#");
     DownloadFile(TabFile[NumFile-1]);   
     Serial.println("\n#END TRANFERT#");
  }
}

// Callback function that download
void OnDownload()
{
  if (VarioCommConnect == false) {
	 VerifySDCard(); 
  }
	
  Serial.println("#BEGIN TRANFERT#");
  for(int i=0;i<compteurFile;i++) {
    Serial.println("#FILE NAME#");
    Serial.println(TabFile[i]);
     Serial.println("#BEGIN FILE TRANFERT#");
     DownloadFile(TabFile[i]);
     Serial.println("\n#END FILE TRANFERT#");
   }
  Serial.println("#END TRANFERT#");
}

void OnExit() 
{
  Serial.println("#GNUVARIO DISCONNECTED#");
	
  statePowerInt = HIGH;	
}

void DownloadFile(String FileName) {
  File myFile;

  if (VarioCommConnect == false) {
	 VerifySDCard(); 
  }

  myFile = SD.open(FileName);
  if (myFile) {
     // read from the file until there's nothing else in it:
     while (myFile.available()) {
       Serial.write(myFile.read());
     }
     // close the file:
     myFile.close();
  } else {
    // if the file didn't open, print an error:
#ifdef PROG_DEBUG 
    Serial.println("error opening File");
#endif //PRO_DEBUG
  }
}

// Show available commands
void ShowCommands() 
{
#ifdef PROG_DEBUG 
  Serial.println("Available commands");
  Serial.println(" 0;                 - This command list");
  Serial.println(" 1;                 - Init / Nb file IGC");
  Serial.println(" 2;                 - List file IGC");
  Serial.println(" 3,<file number>;   - download on file "); 
  Serial.println(" 4;                 - download files "); 
  Serial.println(" 5;                 - disconnect / exit");
#endif //PROG_DEBUG
}

// Setup function
void initVarioComm(void) 
{
  // Listen on serial connection for messages from the PC
  Serial.begin(9600); 
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("#GNUVARIO WAIT CONNECTING#");
 
  // Adds newline to every command
  cmdMessengerUsb.printLfCr();   

  // Attach my application's user-defined callback methods
  attachCommandCallbacks();

  // Show command list
  ShowCommands();
}

// Loop function
void usbConnectedMode(void) 
{
  initVarioComm(); 	
  while (1) {
  // Process incoming serial data, and perform callbacks
    cmdMessengerUsb.feedinSerialData();
	if (statePowerInt == HIGH) {
      statePowerInt = LOW;
      break;
	}
  }
}