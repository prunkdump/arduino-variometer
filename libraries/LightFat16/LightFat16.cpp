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

#include <LightFat16.h>
#include <SdCard.h>

#define BLOCK_SIZE LF16_BLOCK_SIZE

#define FAT16_ID_POS 0x0036
#define MBR_FIRST_PART_POS 0x1be
#define MBR_PART_LBA_POS 0x08
#define BLOCKS_PER_CLUSTER_POS 0x000d
#define RESERVED_BLOCKS_COUNT_POS 0x000e
#define ROOT_ENTRIES_COUNT_POS 0x0011
#define BLOCKS_PER_FAT_POS 0x0016
#define ROOT_ENTRY_SIZE 32
#define ROOT_ENTRY_FREE_TAG 0xe5
#define ROOT_ENTRY_CLUSTER_POS 0x1a
#define ROOT_ENTRY_SIZE_POS 0x1c
#define FAT_ENTRY_SIZE 2
#define FAT_FILENAME_SIZE 8
#define FAT_EOF_TAG 0xffff


void lightfat16::enableSPI(void) {

  card.enableSPI();
}

int lightfat16::init() {

  /****************/
  /* init sd card */
  /****************/
  if( ! card.begin() ) {
    return -1;
  }

  return 0;
}


int lightfat16::begin(void) {

  /* begin with default filename */
  char fileName[] = LF16_DEFAULT_BASE_FILE_NAME;
  return begin(fileName, sizeof(fileName)-1);
}


int lightfat16::begin(char* fileName, uint8_t fileNameSize) {

  /*************************/
  /* find fat 16 partition */
  /*************************/
  uint32_t partitionStartBlock;
  uint8_t* data;
  
  /* ckeck block 0 */
  data = this->blockSet(0, 0);
  if( data[FAT16_ID_POS + 3] == '1' && data[FAT16_ID_POS + 4] == '6' ) { //Check "FAT16"
    partitionStartBlock = 0;
  } else {
    /* read MBR to get the first partition and check again */
    partitionStartBlock = *(uint32_t*)&data[MBR_FIRST_PART_POS + MBR_PART_LBA_POS];
    data = this->blockSet(partitionStartBlock, 0);
    if( data[FAT16_ID_POS + 3] != '1' || data[FAT16_ID_POS + 4] != '6' ) { //Check "FAT16"
      return -1; //no partition found
    }
  }

  /**************************/
  /* read fat 16 parameters */
  /**************************/
  blocksPerCluster = *(uint8_t*)&data[BLOCKS_PER_CLUSTER_POS];
  uint16_t reservedBlocksCount = *(uint16_t*)&data[RESERVED_BLOCKS_COUNT_POS];
  uint16_t rootEntriesCount = *(uint16_t*)&data[ROOT_ENTRIES_COUNT_POS];
  blocksPerFAT = *(uint16_t*)&data[BLOCKS_PER_FAT_POS];
  uint32_t firstFATBlock = partitionStartBlock + reservedBlocksCount;
  uint32_t secondFATBlock = firstFATBlock + blocksPerFAT;
  uint32_t rootDirectoryBlock = secondFATBlock + blocksPerFAT;
  dataBlock = rootDirectoryBlock + rootEntriesCount * 32 / 512;

  /***********************/
  /* find free file name */
  /***********************/
  
  /* load root directory block */
  data = this->blockSet(rootDirectoryBlock, 0);

  for(int fileNumber = 0; fileNumber<LF16_FILE_NAME_NUMBER_LIMIT; fileNumber++) {
  
    /* build file name */
    int digit = fileNumber;
    int base = LF16_FILE_NAME_NUMBER_LIMIT/10;
    for(int i = fileNameSize - LF16_FILE_NAME_NUMBER_SIZE; i<fileNameSize; i++) {
      fileName[i] = '0' + digit/base;
      digit %= base;
      base /= 10;
    }
      
    /* check if is free */
    boolean usedNumber = false;
    while( data[0x00] != 0x00 ) {
      if( data[0x00] != ROOT_ENTRY_FREE_TAG ) {
	boolean nameFound = true;
	for( int i = 0; i<fileNameSize; i++) {
	  if( fileName[i] != data[i] ) {
	    nameFound = false;
	    break;
	  }
	}
	if( nameFound ) {
	  usedNumber = true;
	  break;
	}
      }
      data = this->blockSeek(ROOT_ENTRY_SIZE); //next entry
    }
    /* if free break */
    if( ! usedNumber ) {
      break;
    }
  }

  /************************/
  /* find free root entry */
  /************************/

  data = this->blockSet(rootDirectoryBlock, 0);
  while(data[0x00] != ROOT_ENTRY_FREE_TAG  && data[0x00] != 0x00) {
    data = this->blockSeek(ROOT_ENTRY_SIZE);
  }
  
  this->blockGetPosition(fileEntryBlock, fileEntryPos);

  /*********************/
  /* find free cluster */
  /*********************/
  fileCluster = 0;

  /* search on FAT */
  data = this->blockSet(firstFATBlock, 0);
  while( *(uint16_t*)&data[0] != 0x0000 ) {
    data = this->blockSeek(FAT_ENTRY_SIZE);
    fileCluster++;
  }

  this->blockGetPosition(fileFATBlock, fileFATPos);

  /* compute data block */
  fileDataBlock = dataBlock + (fileCluster - 2)*blocksPerCluster;
  fileDataPos = 0;
  fileClusterBlockUsage = 1;

  /********************/
  /* write file entry */
  /********************/
  data = this->blockSet(fileEntryBlock, fileEntryPos);
  this->blockWriteEnable(false);

  /* write filename */
  for(int i = 0; i<fileNameSize; i++) {
    data[i] = fileName[i];
  }
  for(int i = fileNameSize; i<FAT_FILENAME_SIZE; i++) {
    data[i] = ' ';
  }

  /* write constant part */
  uint8_t fileConstants[] = LF16_FILE_ENTRY_CONSTANTS;
  for(int i=0; i<LF16_FILE_ENTRY_CONSTANTS_SIZE; i++) {
    data[i+8] = fileConstants[i];
  }

  /* write starting cluster */
  *(uint16_t*)&data[ROOT_ENTRY_CLUSTER_POS] = fileCluster;

  /* write file size : 0 */
  *(uint32_t*)&data[ROOT_ENTRY_SIZE_POS] = 0;

  /**************/
  /* write FATs */
  /**************/

  /* first fat */
  data = this->blockSet(fileFATBlock, fileFATPos);
  *(uint16_t*)&data[0] = FAT_EOF_TAG;

  /* second fat */
  data = this->blockSeek(0, blocksPerFAT);
  *(uint16_t*)&data[0] = FAT_EOF_TAG;

  /*********************/
  /* got do data block */
  /* useless to load   */
  /*********************/
  this->blockSet(fileDataBlock, fileDataPos, false);

  return 0;
}

void lightfat16::write(uint8_t inByte) {
  /* write byte */
  blockBuffer[fileDataPos] = inByte;
  fileDataPos++;

  /* check if we need to change of block or cluster */
  if( fileDataPos >= BLOCK_SIZE ) {
    fileDataBlock++;
    fileDataPos = 0;
    fileClusterBlockUsage++;
    if( fileClusterBlockUsage <= blocksPerCluster ) {
      this->fileNewBlock();
    } else {
      fileClusterBlockUsage = 1;
      this->fileNextCluster();
    }
  }
}

void lightfat16::sync() {

  /* moving to root entry, write the current block */
  uint8_t* data = this->blockSet(fileEntryBlock, fileEntryPos);

  /* set size */
  uint32_t currentBlockSize = (*(uint32_t*)&data[ROOT_ENTRY_SIZE_POS]) % BLOCK_SIZE;
  *(uint32_t*)&data[ROOT_ENTRY_SIZE_POS] += fileDataPos - currentBlockSize;

  /* return to current block */
  data = this->blockSet(fileDataBlock, fileDataPos);
}

void lightfat16::fileNewBlock() {

  /* go to root entry to encrease the size of one block */
  /* as write is enabled, this write the current data block */
  uint8_t* data = this->blockSet(fileEntryBlock, fileEntryPos);
  *(uint32_t*)&data[ROOT_ENTRY_SIZE_POS] += BLOCK_SIZE;

  /* go to the new data block */
  /* !!! file dataBlock is already updated !!! */
  /* as write is enabled, this write the file entry */
  /* but useless to load the block data */
  data = this->blockSet(fileDataBlock, fileDataPos, false);

}

void lightfat16::fileNextCluster() {


  /* search on FAT for free cluster */
  uint32_t newFATBlock;
  unsigned newFATPos;
  
  uint8_t* data = this->blockSet(fileFATBlock, fileFATPos);
  while( *(uint16_t*)&data[0] != 0x0000 ) {
    data = this->blockSeek(FAT_ENTRY_SIZE);
    fileCluster++;
  }

  this->blockGetPosition(newFATBlock, newFATPos);

  /* write the next cluster */
  data = this->blockSet(fileFATBlock, fileFATPos);
  *(uint16_t*)&data[0] = fileCluster;

  /* write EOF on the new cluster */
  data = this->blockSet(newFATBlock, newFATPos);
  *(uint16_t*)&data[0] = FAT_EOF_TAG;

  /* same on the second fat */
  data = this->blockSet(fileFATBlock + blocksPerFAT, fileFATPos);
  *(uint16_t*)&data[0] = fileCluster;
  data = this->blockSet(newFATBlock + blocksPerFAT, newFATPos);
  *(uint16_t*)&data[0] = FAT_EOF_TAG;

  /* save new fat */
  fileFATBlock = newFATBlock;
  fileFATPos = newFATPos;
  
  /* compute the new data block */
  fileDataBlock = dataBlock + (fileCluster - 2)*blocksPerCluster;
  fileDataPos = 0;

  /* load new block */
  this->fileNewBlock();
 
}



void lightfat16::blockWriteEnable(boolean reloadBlock) {
  if(reloadBlock) {
    card.readBlock(currentBlock, blockBuffer);
  }

  blockWriteEnabled = true;
}

void lightfat16::blockWriteDisable(boolean writeCurrentChange) {
  if( writeCurrentChange ) {
    card.writeBlock(currentBlock, blockBuffer);
  }

  blockWriteEnabled = false;
}

void lightfat16::blockWriteSync() {
  card.writeBlock(currentBlock, blockBuffer);
}


uint8_t* lightfat16::blockSet(uint32_t block, unsigned pos, boolean loadBlock) {
  /* load block */
  if( block != currentBlock ) {
    if( blockWriteEnabled ) {
      card.writeBlock(currentBlock, blockBuffer);
    }
    currentBlock = block;
    if( loadBlock ) {
      card.readBlock(currentBlock, blockBuffer);
    }
  }

  /* update pos */
  currentPos = pos;
  return &blockBuffer[currentPos];
}

uint8_t* lightfat16::blockSeek(int32_t byteShift, int32_t blockShift, boolean loadBlock) {
  /* seek pos */
  int32_t newPos = currentPos;
  boolean reload = false;
  newPos += byteShift;

  /* write current block if needed */
  if( blockWriteEnabled ) {
    if( newPos >= BLOCK_SIZE || newPos < 0 || blockShift != 0) {
      card.writeBlock(currentBlock, blockBuffer);
    }
  }

  /* make block shift */
  if( blockShift != 0) {
    currentBlock += blockShift;
    reload = true;
  }
  
  /* make byte shift */
  if( newPos >= BLOCK_SIZE || newPos < 0) {
    reload = true;
    currentBlock += newPos / BLOCK_SIZE;
    newPos = newPos % BLOCK_SIZE;
    if( newPos < 0) {
      currentBlock--;
      newPos += BLOCK_SIZE;
    }
  }

  /* load block if needed */
  currentPos = newPos;
  if( reload && loadBlock ) {
    card.readBlock(currentBlock, blockBuffer);
  }
  return &blockBuffer[currentPos];
}

void lightfat16::blockGetPosition(uint32_t& block, unsigned& pos) {
  block = currentBlock;
  pos = currentPos;  
}
