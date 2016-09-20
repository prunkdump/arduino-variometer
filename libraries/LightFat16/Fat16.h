/* Arduino FAT16 Library
 * Copyright (C) 2008 by William Greiman
 *
 * This file is part of the Arduino FAT16 Library
 *
 * This Library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This Library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with the Arduino Fat16 Library.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
#ifndef Fat16_h
#define Fat16_h
/**
 * \file
 * Fat16 class
 */
#include <string.h>
#include <avr/pgmspace.h>
#include <Print.h>
#include <SdCard.h>
#include <utility/FatStructs.h>
#include <Fat16Config.h>
#include <utility/FatApiConstants.h>
//------------------------------------------------------------------------------
/** Fat16 version YYYYMMDD */
#define FAT16_VERSION 20140806

//------------------------------------------------------------------------------
/**
 * \typedef fat_t
 *
 * \brief Type for FAT16 entry
 */
typedef uint16_t fat_t;
/**
 * \union cache16_t
 *
 * \brief Cache buffer data type
 *
 */
union cache16_t {
          /** Used to access cached file data blocks. */
  uint8_t data[512];
          /** Used to access cached FAT entries. */
  fat_t   fat[256];
          /** Used to access cached directory entries. */
  dir_t   dir[16];
          /** Used to access a cached Master Boot Record. */
  mbr_t   mbr;
          /** Used to access to a cached FAT16 boot sector. */
  fat_boot_t   fbs;
};
//------------------------------------------------------------------------------
/** \class Fat16
 * \brief Fat16 implements a minimal Arduino FAT16 Library
 *
 * Fat16 does not support subdirectories or long file names.
 */
class Fat16 : public Print {
 public:
  /*
   * Public functions
   */
  /** create with file closed */
  Fat16(void) : flags_(0) {}
  /** \return The current cluster number. */
  fat_t curCluster(void) const {return curCluster_;}
  bool close(void);
  /** \return The count of clusters in the FAT16 volume. */
  static fat_t clusterCount(void) {return clusterCount_;}
  /** \return The number of 512 byte blocks in a cluster */
  static uint8_t clusterSize(void) {return blocksPerCluster_;}
  /** \return The current file position. */
  uint32_t curPosition(void) const {return curPosition_;}
  /**
   * Set the date/time callback function
   *
   * \param[in] dateTime The user's callback function.  The callback
   * function is of the form:
   *
   * \code
   * void dateTime(uint16_t* date, uint16_t* time) {
   *   uint16_t year;
   *   uint8_t month, day, hour, minute, second;
   *
   *   // User gets date and time from GPS or real-time clock here
   *
   *   // return date using FAT_DATE macro to format fields
   *   *date = FAT_DATE(year, month, day);
   *
   *   // return time using FAT_TIME macro to format fields
   *   *time = FAT_TIME(hour, minute, second);
   * }
   * \endcode
   *
   * Sets the function that is called when a file is created or when
   * a file's directory entry is modified by sync(). All timestamps,
   * access, creation, and modify, are set when a file is created.
   * sync() maintains the last access date and last modify date/time.
   *
   * See the timestamp() function.
   */
  static void dateTimeCallback(
    void (*dateTime)(uint16_t* date, uint16_t* time)) {
    dateTime_ = dateTime;
  }
  /**
   * Cancel the date/time callback function.
   */
  static void dateTimeCallbackCancel(void) {dateTime_ = NULL;}
  bool dirEntry(dir_t* dir);
  int16_t fgets(char* str, int16_t num, char* delim = 0);
  /** \return The file's size in bytes. */
  uint32_t fileSize(void) const {return fileSize_;}
  static bool init(SdCard* dev, uint8_t part);
  /**
   *  Initialize a FAT16 volume.
   *
   *  First try partition 1 then try super floppy format.
   *
   * \param[in] dev The SdCard where the volume is located.
   *
   * \return The value one, true, is returned for success and
   * the value zero, false, is returned for failure.  reasons for
   * failure include not finding a valid FAT16 file system, a call
   * to init() after a volume has been successful initialized or
   * an I/O error.
   *
   */
  static bool init(SdCard* dev) {
    return init(dev, 1) ? true : init(dev, 0);
  }
  /**
   * Checks the file's open/closed status for this instance of Fat16.
   * \return The value true if a file is open otherwise false;
   */
  bool isOpen(void) const {return (flags_ & O_ACCMODE) != 0;}
  static void ls(uint8_t flags = 0);
  bool open(const char* fileName, uint8_t oflag);
  bool open(uint16_t entry, uint8_t oflag);
  static void printDirName(const dir_t& dir, uint8_t width);
  static void printFatDate(uint16_t fatDate);
  static void printFatTime(uint16_t fatTime);
  static void printTwoDigits(uint8_t v);
  int16_t read(void);
  int16_t read(void* buf, uint16_t nbyte);
  static bool readDir(dir_t* dir, uint16_t* index,
                    uint8_t skip = (DIR_ATT_VOLUME_ID | DIR_ATT_DIRECTORY));

  bool remove(void);
  static bool remove(const char* fileName);
  /** Sets the file's current position to zero. */
  void rewind(void) {curPosition_ = curCluster_ = 0;}
  /** \return The number of entries in the root directory. */
  static uint16_t rootDirEntryCount(void) {return rootDirEntryCount_;}
  /**
   * Seek to current position plus \a pos bytes. See Fat16::seekSet(). 
   *
   * \param[in] pos offset from current position.
   *
   * \return true for success or false for failure.
   */
  bool seekCur(int32_t pos) {return seekSet(curPosition_ + pos);}
  /** 
   * Seek to end of file.  See Fat16::seekSet().
   *
   * \return true for success or false for failure.   
   */
  bool seekEnd(void) {return seekSet(fileSize_);}
  bool seekSet(uint32_t pos);
  bool sync(void);
  bool timestamp(uint8_t flag, uint16_t year, uint8_t month, uint8_t day,
          uint8_t hour, uint8_t minute, uint8_t second);
  bool truncate(uint32_t size);
  /** Fat16::writeError is set to true if an error occurs during a write().
   * Set Fat16::writeError to false before calling print() and/or write() and check
   * for true after calls to write() and/or print().
   */
  bool writeError;
  int16_t write(const void *buf, uint16_t nbyte);
  size_t write(uint8_t b);
  int16_t write(const char* str);
  void write_P(PGM_P str);
  void writeln_P(PGM_P str);
//------------------------------------------------------------------------------
#if FAT16_DEBUG_SUPPORT
  /** For debug only.  Do not use in applications. */
  static cache16_t* dbgBufAdd(void) {return &cacheBuffer_;}
  /** For debug only.  Do not use in applications. */
  static void dbgSetDev(SdCard* dev) {rawDev_ = dev;}
  /** For debug only.  Do not use in applications. */
  static uint8_t* dbgCacheBlock(uint32_t blockNumber) {
    return cacheRawBlock(blockNumber) ? cacheBuffer_.data : 0; }
  /** For debug only.  Do not use in applications. */
  static dir_t* dbgCacheDir(uint16_t index) {
    return cacheDirEntry(index);}
#endif  // FAT16_DEBUG_SUPPORT

//------------------------------------------------------------------------------
 private:
  // Volume info
  static bool     volumeInitialized_;  // true if volume has been initialized
  static uint8_t  fatCount_;           // number of FATs
  static uint8_t  blocksPerCluster_;   // must be power of 2
  static uint16_t rootDirEntryCount_;  // should be 512 for FAT16
  static fat_t    blocksPerFat_;       // number of blocks in one FAT
  static fat_t    clusterCount_;       // total clusters in volume
  static uint32_t fatStartBlock_;      // start of first FAT
  static uint32_t rootDirStartBlock_;  // start of root dir
  static uint32_t dataStartBlock_;     // start of data clusters

  // block cache
  static uint8_t const CACHE_FOR_READ  = 0;    // cache a block for read
  static uint8_t const CACHE_FOR_WRITE = 1;    // cache a block and set dirty
  static SdCard *rawDev_;             // Device
  static cache16_t cacheBuffer_;      // 512 byte cache for raw blocks
  static uint32_t cacheBlockNumber_;  // Logical number of block in the cache
  static bool    cacheDirty_;         // cacheFlush() will write block if true
  static uint32_t cacheMirrorBlock_;  // mirror block for second FAT

  // callback function for date/time
  static void (*dateTime_)(uint16_t* date, uint16_t* time);

  // define fields in flags_
  static uint8_t const F_OFLAG = O_ACCMODE | O_APPEND | O_SYNC;
  static uint8_t const F_FILE_DIR_DIRTY = 0X80;  // require sync directory entry

  uint8_t flags_;          // see above for bit definitions
  int16_t dirEntryIndex_;  // index of directory entry for open file
  fat_t firstCluster_;     // first cluster of file
  uint32_t fileSize_;      // fileSize
  fat_t curCluster_;       // current cluster
  uint32_t curPosition_;   // current byte offset

  // private functions for cache
  static uint8_t blockOfCluster(uint32_t position) {
    // depends on blocks per cluster being power of two
    return (position >> 9) & (blocksPerCluster_ - 1);
  }
  static uint16_t cacheDataOffset(uint32_t position) {return position & 0X1FF;}
  static dir_t* cacheDirEntry(uint16_t index, uint8_t action = 0);
  static bool cacheFlush(void);  
  static bool cacheRawBlock(uint32_t blockNumber, uint8_t action = 0);
  static void cacheSetDirty(void) {cacheDirty_ = true;}
  static uint32_t dataBlockLba(fat_t cluster, uint8_t blockOfCluster) {
    return dataStartBlock_ + (uint32_t)(cluster - 2) * blocksPerCluster_
      + blockOfCluster;
  }
  static bool fatGet(fat_t cluster, fat_t* value);
  static bool fatPut(fat_t cluster, fat_t value);
  // end of chain test
  static bool isEOC(fat_t cluster) {return cluster >= 0XFFF8;}
  // allocate a cluster to a file
  bool addCluster(void);
  // free a cluster chain
  bool freeChain(fat_t cluster);
};
#endif  // Fat16_h
