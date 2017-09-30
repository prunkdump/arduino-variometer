
#include <VarioComm.h>
#include <Arduino.h>
#include <VarioSettings.h>
#include <SD.h>

#define USB Serial


//==============================================================================================================================================
void VarioComm::setup() {
  Serial.begin(9600);                                 // Setup serial communication with the computer (using a baud rate of 9600 on serial monitor)
  pinMode(LED,OUTPUT);                                // Define digital pin 13 as an OUTPUT pin - so that we can use it with an LED
  digitalWrite(LED,LOW);                              // Turn off the LED
}

//================================================================================================================================================
void VarioComm::Execute() {
  if(Serial.available()){
    computerByte = Serial.read();                      //read any incoming bytes from the Serial monitor, and store this byte in the variable called computerByte
    if(computerByte==49){               //1            //If you send the number 1 from the serial monitor, the arduino will read it as digital number 49. Google "ascii table" for more info.
      printCommandHeader("COMMAND1: CHECK CONNECTION");
      checkConnection(0x01);                           // Check for successful connection and communication with the CH376S module.
    } 
    if(computerByte==50){               //2
     printCommandHeader("COMMAND2: set_USB_Mode");
      set_USB_Mode(0x06);                              // Code used to enable read/write communication and monitoring of the USB stick
    }
    if(computerByte==51){               //3
      printCommandHeader("COMMAND3: resetALL");
      resetALL();                                      // Reset the USB device
    }
    if(computerByte==52){               //4
      printCommandHeader("COMMAND4: Create and Write to File : TEST4.TXT");
      writeFile("TEST4.TXT", wrData);                  // Create a file called TEST4.TXT and then Write the contents of wrData to this file
    }
    if(computerByte==53){               //5
      printCommandHeader("COMMAND5: Read File: TEST4.TXT");
      readFile("TEST4.TXT");                           // Read the contents of this file on the USB disk, and display contents in the Serial Monitor
    }
    if(computerByte==54){               //6
      printCommandHeader("COMMAND6: Append data to file: TEST4.TXT");
      appendFile("TEST4.TXT", wrData2);                // Append data to the end of the file.
    }
    if(computerByte==55){               //7
      printCommandHeader("COMMAND7: Delete File: TEST4.TXT");
      fileDelete("TEST4.TXT");                         // Delete the file named TEST4.TXT
    }
    if(computerByte==56){               //8
      printCommandHeader("COMMAND8: Read File: TEST2.TXT");
      readFile("TEST2.TXT");                           // Read the contents of the TEST2.TXT file on the USB disk, and display contents in the Serial Monitor
    }
    if(computerByte==57){               //9
      printCommandHeader("COMMAND9: Read File: TEST3.TXT");
      readFile("TEST3.TXT");                           // Read the contents of the TEST3.TXT file on the USB disk, and display contents in the Serial Monitor
    }
  }
  
  if(USB.available()){                                 // This is here to capture any unexpected data transmitted by the CH376S module
    Serial.print("CH376S has just sent this code:");
    Serial.println(USB.read(), HEX);
  }
}

//END OF LOOP FUNCTION ========================================================================================================================================

//print Command header
void VarioComm::printCommandHeader(String header){
   Serial.println("======================");
   Serial.println("");
   Serial.println(header);
   Serial.println("----------------------");
}

//checkConnection==================================================================================
//This function is used to check for successful communication with the PC. 
//Send any value between 0 to 255, and the PC will return a number = 255 - value. 
void VarioComm::checkConnection(byte value){
  Serial.write(0x57);
  Serial.write(0xAB);
  Serial.write(0x06);
  Serial.write(value);
  
  if(waitForResponse("checking connection")){       //wait for a response from the CH376S. If CH376S responds, it will be true. If it times out, it will be false.
    if(getResponseFromUSB()==(255-value)){
       Serial.println(">Connection to CH376S was successful.");
       blinkLED();                               //blink the LED for 1 second if the connection was successful
    } else {
      Serial.print(">Connection to CH376S - FAILED.");
    }
  }
}

//set_USB_Mode=====================================================================================
//Make sure that the USB is inserted when using 0x06 as the value in this specific code sequence
void VarioComm::set_USB_Mode (byte value){
  USB.write(0x57);
  USB.write(0xAB);
  USB.write(0x15);
  USB.write(value);
  
  delay(20);
  
  if(USB.available()){
    USB_Byte=USB.read();
    //Check to see if the command has been successfully transmitted and acknowledged.
    if(USB_Byte==0x51){                                   // If true - the CH376S has acknowledged the command.
        Serial.println("set_USB_Mode command acknowledged"); //The CH376S will now check and monitor the USB port
        USB_Byte = USB.read();
        
        //Check to see if the USB stick is connected or not.
        if(USB_Byte==0x15){                               // If true - there is a USB stick connected
          Serial.println("USB is present");
          blinkLED();                                     // If the process was successful, then turn the LED on for 1 second 
        } else {
          Serial.print("USB Not present. Error code:");   // If the USB is not connected - it should return an Error code = FFH
          Serial.print(USB_Byte, HEX);
          Serial.println("H");
        }
        
    } else {
        Serial.print("CH3765 error!   Error code:");
        Serial.print(USB_Byte, HEX);
        Serial.println("H");
    }   
  }
  delay(20);
}

//resetALL=========================================================================================
//This will perform a hardware reset of the CH376S module - which usually takes about 35 msecs =====
void VarioComm::resetALL(){
    USB.write(0x57);
    USB.write(0xAB);
    USB.write(0x05);
    Serial.println("The CH376S module has been reset !");
    delay(200);
}

//readFile=====================================================================================
//This will send a series of commands to read data from a specific file (defined by fileName)
void VarioComm::readFile(String fileName){
  resetALL();                     //Reset the module
  set_USB_Mode(0x06);             //Set to USB Mode
  diskConnectionStatus();         //Check that communication with the USB device is possible
  USBdiskMount();                 //Prepare the USB for reading/writing - you need to mount the USB disk for proper read/write operations.
  setFileName(fileName);          //Set File name
  fileOpen();                     //Open the file for reading
  int fs = getFileSize();         //Get the size of the file
  fileRead();                     //***** Send the command to read the file ***
  fileClose(0x00);                //Close the file
}

//writeFile========================================================================================
//is used to create a new file and then write data to that file. "fileName" is a variable used to hold the name of the file (e.g TEST.TXT). "data" should not be greater than 255 bytes long. 
void VarioComm::writeFile(String fileName, String data){
  resetALL();                     //Reset the module
  set_USB_Mode(0x06);             //Set to USB Mode
  diskConnectionStatus();         //Check that communication with the USB device is possible
  USBdiskMount();                 //Prepare the USB for reading/writing - you need to mount the USB disk for proper read/write operations.
  setFileName(fileName);          //Set File name
  if(fileCreate()){               //Try to create a new file. If file creation is successful
    fileWrite(data);              //write data to the file.
  } else {
    Serial.println("File could not be created, or it already exists");
  }
  fileClose(0x01);
}

//appendFile()====================================================================================
//is used to write data to the end of the file, without erasing the contents of the file.
void VarioComm::appendFile(String fileName, String data){
    resetALL();                     //Reset the module
    set_USB_Mode(0x06);             //Set to USB Mode
    diskConnectionStatus();         //Check that communication with the USB device is possible
    USBdiskMount();                 //Prepare the USB for reading/writing - you need to mount the USB disk for proper read/write operations.
    setFileName(fileName);          //Set File name
    fileOpen();                     //Open the file
    filePointer(false);             //filePointer(false) is to set the pointer at the end of the file.  filePointer(true) will set the pointer to the beginning.
    fileWrite(data);                //Write data to the end of the file
    fileClose(0x01);                //Close the file using 0x01 - which means to update the size of the file on close. 
}
  
//setFileName======================================================================================
//This sets the name of the file to work with
void VarioComm::setFileName(String fileName){
  Serial.print("Setting filename to:");
  Serial.println(fileName);
  USB.write(0x57);
  USB.write(0xAB);
  USB.write(0x2F);
  USB.write(0x2F);         // Every filename must have this byte to indicate the start of the file name.
  USB.print(fileName);     // "fileName" is a variable that holds the name of the file.  eg. TEST.TXT
  USB.write((byte)0x00);   // you need to cast as a byte - otherwise it will not compile.  The null byte indicates the end of the file name.
  delay(20);
}

//diskConnectionStatus================================================================================
//Check the disk connection status
void VarioComm::diskConnectionStatus(){
  Serial.println("Checking USB disk connection status");
  USB.write(0x57);
  USB.write(0xAB);
  USB.write(0x30);

  if(waitForResponse("Connecting to USB disk")){       //wait for a response from the CH376S. If CH376S responds, it will be true. If it times out, it will be false.
    if(getResponseFromUSB()==0x14){               //CH376S will send 0x14 if this command was successful
       Serial.println(">Connection to USB OK");
    } else {
      Serial.print(">Connection to USB - FAILED.");
    }
  }
}

//USBdiskMount========================================================================================
//initialise the USB disk and check that it is ready - this process is required if you want to find the manufacturing information of the USB disk
void VarioComm::USBdiskMount(){
  Serial.println("Mounting USB disk");
  USB.write(0x57);
  USB.write(0xAB);
  USB.write(0x31);

  if(waitForResponse("mounting USB disk")){       //wait for a response from the CH376S. If CH376S responds, it will be true. If it times out, it will be false.
    if(getResponseFromUSB()==0x14){               //CH376S will send 0x14 if this command was successful
       Serial.println(">USB Mounted - OK");
    } else {
      Serial.print(">Failed to Mount USB disk.");
    }
  }
}

//fileOpen========================================================================================
//opens the file for reading or writing
void VarioComm::fileOpen(){
  Serial.println("Opening file.");
  USB.write(0x57);
  USB.write(0xAB);
  USB.write(0x32);
  if(waitForResponse("file Open")){                 //wait for a response from the CH376S. If CH376S responds, it will be true. If it times out, it will be false.
    if(getResponseFromUSB()==0x14){                 //CH376S will send 0x14 if this command was successful  
       Serial.println(">File opened successfully.");
    } else {
      Serial.print(">Failed to open file.");
    }
  }
}

//setByteRead=====================================================================================
//This function is required if you want to read data from the file. 
boolean VarioComm::setByteRead(byte numBytes){
  boolean bytesToRead=false;
  int timeCounter = 0;
  USB.write(0x57);
  USB.write(0xAB);
  USB.write(0x3A);
  USB.write((byte)numBytes);   //tells the CH376S how many bytes to read at a time
  USB.write((byte)0x00);
  if(waitForResponse("setByteRead")){       //wait for a response from the CH376S. If CH376S responds, it will be true. If it times out, it will be false.
    if(getResponseFromUSB()==0x1D){         //read the CH376S message. If equal to 0x1D, data is present, so return true. Will return 0x14 if no data is present.
      bytesToRead=true;
    }
  }
  return(bytesToRead);
} 

//getFileSize()===================================================================================
//writes the file size to the serial Monitor.
int VarioComm::getFileSize(){
  int fileSize=0;
  Serial.println("Getting File Size");
  USB.write(0x57);
  USB.write(0xAB);
  USB.write(0x0C);
  USB.write(0x68);
  delay(100);
  Serial.print("FileSize =");
  if(USB.available()){
    fileSize = fileSize + USB.read();
  } 
  if(USB.available()){
    fileSize = fileSize + (USB.read()*255);
  } 
  if(USB.available()){
    fileSize = fileSize + (USB.read()*255*255);
  } 
  if(USB.available()){
    fileSize = fileSize + (USB.read()*255*255*255);
  }     
  Serial.println(fileSize);
  delay(10);
  return(fileSize);
}


//fileRead========================================================================================
//read the contents of the file
void VarioComm::fileRead(){
  Serial.println("Reading file:");
  byte firstByte = 0x00;                     //Variable to hold the firstByte from every transmission.  Can be used as a checkSum if required.
  byte numBytes = 0x40;                      //The maximum value is 0x40  =  64 bytes
 
  while(setByteRead(numBytes)){              //This tells the CH376S module how many bytes to read on the next reading step. In this example, we will read 0x10 bytes at a time. Returns true if there are bytes to read, false if there are no more bytes to read.
    USB.write(0x57);
    USB.write(0xAB);
    USB.write(0x27);                          //Command to read ALL of the bytes (allocated by setByteRead(x))
    if(waitForResponse("reading data")){      //Wait for the CH376S module to return data. TimeOut will return false. If data is being transmitted, it will return true.
        firstByte=USB.read();                 //Read the first byte
        while(USB.available()){
          Serial.write(USB.read());           //Send the data from the USB disk to the Serial monitor
          delay(1);                           //This delay is necessary for successful Serial transmission
        }
    }
    if(!continueRead()){                       //prepares the module for further reading. If false, stop reading.
      break;                                   //You need the continueRead() method if the data to be read from the USB device is greater than numBytes.
    }
  }
  Serial.println();
  Serial.println("NO MORE DATA");
}

//fileWrite=======================================================================================
//are the commands used to write to the file
void VarioComm::fileWrite(String data){
  Serial.println("Writing to file:");
  byte dataLength = (byte) data.length();         // This variable holds the length of the data to be written (in bytes)
  Serial.println(data);
  Serial.print("Data Length:");
  Serial.println(dataLength);
  delay(100);
  // This set of commands tells the CH376S module how many bytes to expect from the Arduino.  (defined by the "dataLength" variable)
  USB.write(0x57);
  USB.write(0xAB);
  USB.write(0x3C);
  USB.write((byte) dataLength);
  USB.write((byte) 0x00);
  if(waitForResponse("setting data Length")){      // Wait for an acknowledgement from the CH376S module before trying to send data to it
    if(getResponseFromUSB()==0x1E){                // 0x1E indicates that the USB device is in write mode.
      USB.write(0x57);
      USB.write(0xAB);
      USB.write(0x2D);
      USB.print(data);                             // write the data to the file
  
      if(waitForResponse("writing data to file")){   // wait for an acknowledgement from the CH376S module
      }
      Serial.print("Write code (normally FF and 14): ");
      Serial.print(USB.read(),HEX);                // code is normally 0xFF
      Serial.print(",");
      USB.write(0x57);
      USB.write(0xAB);
      USB.write(0x3D);                             // This is used to update the file size. Not sure if this is necessary for successful writing.
      if(waitForResponse("updating file size")){   // wait for an acknowledgement from the CH376S module
      }
      Serial.println(USB.read(),HEX);              //code is normally 0x14
    }
  }
}

//continueRead()==================================================================================
//continue to read the file : I could not get this function to work as intended.
boolean VarioComm::continueRead(){
  boolean readAgain = false;
  USB.write(0x57);
  USB.write(0xAB);
  USB.write(0x3B);
  if(waitForResponse("continueRead")){       //wait for a response from the CH376S. If CH376S responds, it will be true. If it times out, it will be false.
     if(getResponseFromUSB()==0x14){         //CH376S will send 0x14 if this command was successful
       readAgain=true;
     }
  }
  return(readAgain);
} 

//fileCreate()========================================================================================
//the command sequence to create a file
boolean VarioComm::fileCreate(){
  boolean createdFile = false;
  USB.write(0x57);
  USB.write(0xAB);
  USB.write(0x34);
  if(waitForResponse("creating file")){       //wait for a response from the CH376S. If file has been created successfully, it will return true.
     if(getResponseFromUSB()==0x14){          //CH376S will send 0x14 if this command was successful
       createdFile=true;
     }
  }
  return(createdFile);
}


//fileDelete()========================================================================================
//the command sequence to delete a file
void VarioComm::fileDelete(String fileName){
  setFileName(fileName);
  delay(20);
  USB.write(0x57);
  USB.write(0xAB);
  USB.write(0x35);
  if(waitForResponse("deleting file")){       //wait for a response from the CH376S. If file has been created successfully, it will return true.
     if(getResponseFromUSB()==0x14){          //CH376S will send 0x14 if this command was successful
       Serial.println("Successfully deleted file");
     }
  }
}
  

//filePointer========================================================================================
//is used to set the file pointer position. true for beginning of file, false for the end of the file.
void VarioComm::filePointer(boolean fileBeginning){
  USB.write(0x57);
  USB.write(0xAB);
  USB.write(0x39);
  if(fileBeginning){
    USB.write((byte)0x00);             //beginning of file
    USB.write((byte)0x00);
    USB.write((byte)0x00);
    USB.write((byte)0x00);
  } else {
    USB.write((byte)0xFF);             //end of file
    USB.write((byte)0xFF);
    USB.write((byte)0xFF);
    USB.write((byte)0xFF);
  }
  if(waitForResponse("setting file pointer")){       //wait for a response from the CH376S. 
     if(getResponseFromUSB()==0x14){                 //CH376S will send 0x14 if this command was successful
       Serial.println("Pointer successfully applied");
     }
  }
}


//fileClose=======================================================================================
//closes the file
void VarioComm::fileClose(byte closeCmd){
  Serial.println("Closing file:");
  USB.write(0x57);
  USB.write(0xAB);
  USB.write(0x36);
  USB.write((byte)closeCmd);                                // closeCmd = 0x00 = close without updating file Size, 0x01 = close and update file Size

  if(waitForResponse("closing file")){                      // wait for a response from the CH376S. 
     byte resp = getResponseFromUSB();
     if(resp==0x14){                                        // CH376S will send 0x14 if this command was successful
       Serial.println(">File closed successfully.");
     } else {
       Serial.print(">Failed to close file. Error code:");
       Serial.println(resp, HEX);
     }  
  }
}

//waitForResponse===================================================================================
//is used to wait for a response from USB. Returns true when bytes become available, false if it times out.
boolean VarioComm::waitForResponse(String errorMsg){
  boolean bytesAvailable = true;
  int counter=0;
  while(!Serial.available()){     //wait for PC to verify command
    delay(1);
    counter++;
    if(counter>timeOut){
      Serial.print("TimeOut waiting for response: Error while: ");
      Serial.println(errorMsg);
      bytesAvailable = false;
      break;
    }
  }
  delay(1);
  return(bytesAvailable);
}

//waitForConnection===================================================================================
//is used to wait for a response from USB. Returns true when bytes become available, false if it times out.
boolean VarioComm::waitForConnection(){
  boolean bytesAvailable = true;
  int counter=0;
  while(!Serial){     //wait for connection
    delay(1);
    counter++;
    if(counter>timeOut){
      bytesAvailable = false;
      break;
    }
  }
  delay(1);
  return(bytesAvailable);
}

//getResponseFromUSB================================================================================
//is used to get any error codes or messages from the PC (in response to certain commands)
byte VarioComm::getResponseFromUSB(){
  byte response = byte(0x00);
  if (Serial.available()){
    response = Serial.read();
  }
  return(response);
}



//blinkLED==========================================================================================
//Turn an LED on for 1 second
void VarioComm::blinkLED(){
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED,LOW);
}


/*
const int chipSelect = SDCARD_SS_PIN;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("SETTINGS.TXT");

  // if the file is available, write to it:
  if (dataFile) {
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}*/