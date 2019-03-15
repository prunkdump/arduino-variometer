/* InTW -- Interrupt driven Two Wire library 
 *
 * Copyright 2016-2019 Baptiste PELLEGRIN
 * 
 * This file is part of GNUVario.
 *
 * GNUVario is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * GNUVario is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef INT_TW_H
#define INT_TW_H

#include <Arduino.h>

/*************************************/
/* enable / disable functionnalities */
/*************************************/
//#define INTTW_LOAD_POINTER_FUNC
//#define INTTW_CALLBACK_ON_ERROR_FUNC

/**************************************************************/
/*                 IntTW library                              */
/*                                                            */
/*  This library execute Two Wires requests from commands     */
/*  stored in buffer. All the communication is managed by     */
/*  interrupts so many other operations can be done in        */
/*  the same time.                                            */
/*                                                            */
/*  The command buffer is a sequence of requests              */
/*  stored like this :                                        */
/*                                                            */
/*  1) One byte for device's address and the operation.       */
/*    ex : INTTW_ACTION(myDeviceAddress, INTTW_WRITE)         */
/*                                                            */
/*  2) One byte for the number of bytes to read or            */
/*     write and where to find/store them.                    */
/*    ex : INTTW_DEST(count, INTTW_AT_POINTER)                */
/*                                                            */
/*  3) (optionnal) The dest/receive buffer pointer            */
/*     or the bytes that need to be transfered/received.      */
/*                                                            */
/* Here an example of command buffer that write 0x22, 0x33    */
/* to two devices and read 5 bytes from one  :                */
/*                                                            */
/* uint8_t myCmd[] = { INTTW_ACTION(myAddress1, INTTW_WRITE), */
/*                     INTTW_DEST(2, INTTW_IN_CMD),           */
/*                     0x22, 0x33,                            */
/*                     INTTW_ACTION(myAddress2, INTTW_WRITE), */
/*                     INTTW_DEST(2, INTTW_IN_CMD),           */
/*                     0x22, 0x33,                            */
/*                     INTTW_ACTION(myAddress1, INTTW_READ),  */
/*                     INTTW_DEST(5, INTTW_AT_POINTER) }      */
/*                                                            */
/* See details below.                                         */ 
/**************************************************************/


/*******************************/
/* The Two Wire action byte    */
/* -> the device               */
/* -> the operation (R or W)   */
/*******************************/
#define INTTW_ACTION(address, rw) (address << 1) | rw
#define INTTW_WRITE 0x00
#define INTTW_READ 0x01


/************************************************/
/* The destination byte                         */
/* -> Numbers of bytes to send/receive (max 63) */
/* -> Where to find or store them               */
/************************************************/
#define INTTW_DEST(count, dest)  count | dest

//transmitted from/to the command buffer (just after the count byte)  
#define INTTW_IN_CMD 0x00

//transmitted from/to the current tx/rx pointer (see setTxBuffer/setRxBuffer)
#define INTTW_AT_POINTER 0x80

#ifdef INTTW_LOAD_POINTER_FUNC
//transmitted from/to the pointer stored just after the count byte  
#define INTTW_SET_POINTER 0xC0
#endif

/* the destination type is stored in the count byte */
#define INTTW_COUNT_MASK 0x3F
#define INTTW_COUNT_FLAG_MASK 0xC0
#define INTTW_COUNT_FLAG_USE_POINTER 0x80

/*************/
/* The flags */
/*************/
#define INTTW_NONE 0

//Read the cmd from progmem (you can't use writes with INTTW_IN_CMD)
#define INTTW_USE_PROGMEM 1

//Don't release the Two Wire bus. You need to launch another
//command immediately after. The last command need to
//have this flag disabled
//!!! Don't keep bus for multiple read (burst) on the same device, this is not implemented !!!
#define INTTW_KEEP_BUS 2

class IntTW {

 public:
 IntTW(void) : flags(0), count(0) { }
  void begin(bool internalPullUp = true);

  /* be sure than the previous sequence is terminated */
  /* before starting a new one                        */
  void setTxBuffer(uint8_t* buff);
  void setRxBuffer(uint8_t* buff);
  void start(uint8_t* commands, uint8_t commandLength, uint8_t commandflags = INTTW_NONE, void (*successCallback)(void) = NULL); //callback called also on error

  /* sequence control */
  bool transmitting(void);
  bool succeeded(void);
  void stop(void); //force stop in case of keeping bus

  /* blocking methods below (wait for transfert finished) */
  // return true on success
  bool writeBytes(uint8_t address, uint8_t cmd, uint8_t count, uint8_t* buff);
  bool readBytes(uint8_t address, uint8_t cmd, uint8_t count, uint8_t* buff);

  void twiVect(void); //twi interrupt
  
 private:
  void checkStopOrRestart(void);
  bool canContinue(uint8_t* nextCmdPtr);
  void parseAction(void);
  void transmitByte(void);
  uint8_t volatile flags;
  uint8_t volatile currentAddress;
  uint8_t volatile count;
  uint8_t volatile expectedState;
  uint8_t volatile * volatile cmd;
  uint8_t* cmdStart;
  uint8_t cmdLength;
  uint8_t* volatile txBuffer;
  uint8_t volatile * volatile rxBuffer;
  void (*callback)(void); 
};
  
extern IntTW intTW;  

#endif //INT_TW_H
