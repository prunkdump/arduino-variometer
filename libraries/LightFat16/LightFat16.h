/* LightFat16 -- extra light SD card and Fat16 library
 *
 * Based on sdfatlib by William Greiman
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

#ifndef LIGHT_FAT_16_H
#define LIGHT_FAT_16_H

#include <Arduino.h>
#include <SdCard.h>
#include <VarioSettings.h>

/***************************************/
/* You can compile with static CS pin. */
/* For this define :                   */
/*                                     */
/* #define SDCARD_CS_PIN               */
/***************************************/
static constexpr uint8_t lightFat16DefaultPin = SS;

#define LF16_FILE_NAME_NUMBER_SIZE 2
#define LF16_FILE_NAME_NUMBER_LIMIT 100
#define LF16_DEFAULT_BASE_FILE_NAME "GPS00"

/* the first 3 characters give the extension */
/* ex : TXT -> 0x54,0x58,0x54 */
#define LF16_FILE_ENTRY_CONSTANTS {0x49,0x47,0x43,0x20,0x00,0x64,0xD0,0x89,0x26,0x49,0x26,0x49,0x00,0x00,0xD0,0x89,0x26,0x49}
#define LF16_FILE_ENTRY_CONSTANTS_SIZE 18

#define LF16_BLOCK_SIZE 512

class lightfat16 {

 public :
#ifndef SDCARD_CS_PIN
 lightfat16(uint8_t csPin = lightFat16DefaultPin) : card(csPin), currentBlock(-1), blockWriteEnabled(false) { }
#else
 lightfat16() : currentBlock(-1), blockWriteEnabled(false) { }
#endif
  void enableSPI(void); //set just the CS line
  int init(void);       //just search for SD card
  int begin(void);      // create file
  int begin(char* fileName, uint8_t fileNameSize); //!! last bytes are used for the incrementing number
  void write(uint8_t inByte);
  void sync();

 private:
  SdCard card;
  uint32_t currentBlock;
  unsigned currentPos;
  uint8_t blockBuffer[LF16_BLOCK_SIZE];
  boolean blockWriteEnabled;

  uint8_t blocksPerCluster;
  uint16_t blocksPerFAT;
  uint32_t dataBlock;
  uint32_t fileEntryBlock;
  unsigned fileEntryPos;
  uint32_t fileFATBlock;
  unsigned fileFATPos;
  unsigned fileCluster;
  uint32_t fileDataBlock;
  unsigned fileDataPos;
  uint8_t fileClusterBlockUsage;

  void fileNewBlock();
  void fileNextCluster();
  void blockWriteEnable(boolean reloadBlock = true);
  void blockWriteDisable(boolean writeCurrentChange = true);
  void blockWriteSync();
  uint8_t* blockSet(uint32_t block, unsigned pos, boolean loadBlock = true);
  uint8_t* blockSeek(int32_t byteShift, int32_t blockShift = 0, boolean loadBlock = true);
  void blockGetPosition(uint32_t& block, unsigned& pos);
  
};


#endif
