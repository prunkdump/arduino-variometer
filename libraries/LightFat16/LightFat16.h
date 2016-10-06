#ifndef LIGHT_FAT_16_H
#define LIGHT_FAT_16_H

#include <SdCard.h>

#define FILE_NAME_NUMBER_SIZE 3
#define FILE_NAME_NUMBER_LIMIT 1000
#define BASE_FILE_NAME "GPS000"

#define BLOCK_SIZE 512
#define FILE_ENTRY_CONSTANTS {0x54,0x58,0x54,0x20,0x00,0x64,0xD0,0x89,0x26,0x49,0x26,0x49,0x00,0x00,0xD0,0x89,0x26,0x49}
#define FILE_ENTRY_CONSTANTS_SIZE 18


class lightfat16 {

 public :
  lightfat16();
  int init(int sspin, uint8_t sckDivisor = SPI_CLOCK_DIV2);
  void write(uint8_t inByte);
  void sync();
  

 private:
  SdCard card;
  uint32_t currentBlock;
  unsigned currentPos;
  uint8_t blockBuffer[BLOCK_SIZE];
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
