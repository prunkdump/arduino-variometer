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
#ifndef SdCard_h
#define SdCard_h
 /**
  * \file
  * SdCard class
  */
#include <Arduino.h>
#include <SPI.h>

//------------------------------------------------------------------------------
// SD operation timeouts
/** init timeout ms */
uint16_t const SD_INIT_TIMEOUT = 2000;
/** erase timeout ms */
uint16_t const SD_ERASE_TIMEOUT = 10000;
/** read timeout ms */
uint16_t const SD_READ_TIMEOUT = 300;
/** write time out ms */
uint16_t const SD_WRITE_TIMEOUT = 600;
//------------------------------------------------------------------------------
// SD card commands
/** GO_IDLE_STATE - init card in spi mode if CS low */
uint8_t const CMD0 = 0X00;
/** SEND_IF_COND - verify SD Memory Card interface operating condition.*/
uint8_t const CMD8 = 0X08;
/** SEND_CSD - read the Card Specific Data (CSD register) */
uint8_t const CMD9 = 0X09;
/** SEND_CID - read the card identification information (CID register) */
uint8_t const CMD10 = 0X0A;
/** STOP_TRANSMISSION - end multiple block read sequence */
uint8_t const CMD12 = 0X0C;
/** SEND_STATUS - read the card status register */
uint8_t const CMD13 = 0X0D;
/** READ_SINGLE_BLOCK - read a single data block from the card */
uint8_t const CMD17 = 0X11;
/** READ_MULTIPLE_BLOCK - read a multiple data blocks from the card */
uint8_t const CMD18 = 0X12;
/** WRITE_BLOCK - write a single data block to the card */
uint8_t const CMD24 = 0X18;
/** WRITE_MULTIPLE_BLOCK - write blocks of data until a STOP_TRANSMISSION */
uint8_t const CMD25 = 0X19;
/** ERASE_WR_BLK_START - sets the address of the first block to be erased */
uint8_t const CMD32 = 0X20;
/** ERASE_WR_BLK_END - sets the address of the last block of the continuous
    range to be erased*/
uint8_t const CMD33 = 0X21;
/** ERASE - erase all previously selected blocks */
uint8_t const CMD38 = 0X26;
/** APP_CMD - escape for application specific command */
uint8_t const CMD55 = 0X37;
/** READ_OCR - read the OCR register of a card */
uint8_t const CMD58 = 0X3A;
/** CRC_ON_OFF - enable or disable CRC checking */
uint8_t const CMD59 = 0X3B;
/** SET_WR_BLK_ERASE_COUNT - Set the number of write blocks to be
     pre-erased before writing */
uint8_t const ACMD23 = 0X17;
/** SD_SEND_OP_COMD - Sends host capacity support information and
    activates the card's initialization process */
uint8_t const ACMD41 = 0X29;
//------------------------------------------------------------------------------
/** status for card in the ready state */
uint8_t const R1_READY_STATE = 0X00;
/** status for card in the idle state */
uint8_t const R1_IDLE_STATE = 0X01;
/** status bit for illegal command */
uint8_t const R1_ILLEGAL_COMMAND = 0X04;
/** start data token for read or write single block*/
uint8_t const DATA_START_BLOCK = 0XFE;
/** stop token for write multiple blocks*/
uint8_t const STOP_TRAN_TOKEN = 0XFD;
/** start data token for write multiple blocks*/
uint8_t const WRITE_MULTIPLE_TOKEN = 0XFC;
/** mask for data response tokens after a write block operation */
uint8_t const DATA_RES_MASK = 0X1F;
/** write data accepted token */
uint8_t const DATA_RES_ACCEPTED = 0X05;
//------------------------------------------------------------------------------
// error codes
/** Card did not go into SPI mode */
uint8_t const SD_ERROR_CMD0              = 0X1;
/** Card did not go ready */
uint8_t const SD_ERROR_ACMD41            = 0X2;
/** Write command not accepted */
uint8_t const SD_ERROR_CMD24             = 0X3;
/** Read command not accepted */
uint8_t const SD_ERROR_CMD17             = 0X4;
/** timeout waiting for read data */
uint8_t const SD_ERROR_READ_TIMEOUT      = 0X5;
/** write error occurred */
uint8_t const SD_ERROR_WRITE_RESPONSE    = 0X6;
/** timeout waiting for write status */
uint8_t const SD_ERROR_WRITE_TIMEOUT     = 0X7;
/** attempt to write block zero */
uint8_t const SD_ERROR_BLOCK_ZERO_WRITE  = 0X8;
/** card returned an error to a CMD13 status check after a write */
uint8_t const SD_ERROR_WRITE_PROGRAMMING = 0X9;
/** card fialed to initialize with CMD1*/
uint8_t const SD_ERROR_CMD1              = 0XA;

//------------------------------------------------------------------------------
/**
 * \class SdCard
 * \brief Hardware access class for SD flash cards
 *
 * Supports raw access to a standard SD flash memory card.
 *
 */
class SdCard  {
 public:
  /** Code for a SD error. See SdCard.h for definitions. */
  uint8_t errorCode;
  /** Data that may be helpful in determining the cause of an error */
  uint8_t errorData;
  
  bool begin(uint8_t chipSelect = SS, uint8_t sckDivisor = SPI_CLOCK_DIV2);
  /**
   * Initialize an SD flash memory card with default clock rate and chip
   * select pin.  See SdCard::begin(uint8_t chipSelectPin, uint8_t sckRateID).
   *
   * \return true for success or false for failure.
   */
  bool init(void) {
    return begin(SS, SPI_CLOCK_DIV2);
  }
  /**
   * Initialize an SD flash memory card.
   * 
   * \param[in] halfSpeed set SCK rate to half speed if true else full speed.  
   *
   * \return true for success or false for failure.   
   */
  bool init(bool halfSpeed) {
    return begin(halfSpeed ? SPI_CLOCK_DIV4 : SPI_CLOCK_DIV2, SS);
  }
  /**
   * Initialize an SD flash memory card.
   *
   * \param[in] halfSpeed set SCK rate to half speed if true else full speed.
   * \param[in] chipSelect SD card chip select pin.
   *
   * \return true for success or false for failure.
   */  
  bool init(bool halfSpeed, uint8_t chipSelect) {
    return begin(halfSpeed ? SPI_CLOCK_DIV4 : SPI_CLOCK_DIV2, chipSelect);}
  bool readBlock(uint32_t block, uint8_t* dst);
  bool writeBlock(uint32_t block, const uint8_t* src);
 private:
  uint8_t cardAcmd(uint8_t cmd, uint32_t arg);
  uint8_t cardCommand(uint8_t cmd, uint32_t arg);
  uint8_t chipSelectPin_;
  void chipSelectHigh(void);
  void chipSelectLow(void);
  void error(uint8_t code, uint8_t data);
  void error(uint8_t code);
  bool readReg(uint8_t cmd, void* buf);
  bool readTransfer(uint8_t* dst, uint16_t count);
};
#endif  // SdCard_h
