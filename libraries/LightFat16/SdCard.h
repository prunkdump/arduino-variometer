/* LightFat16 -- extra light SD card and Fat16 library
 *
 * Based on sdfatlib by William Greiman
 * Copyright (C) 2008 by William Greiman
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

#ifndef SdCard_h
#define SdCard_h
 /**
  * \file
  * SdCard class
  */
#include <Arduino.h>
#include <SPI.h>
#include <VarioSettings.h>

/***************************************/
/* You can compile with static CS pin. */
/* For this define :                   */
/*                                     */
/* #define SDCARD_CS_PIN               */
/***************************************/
static constexpr uint8_t sdCardDefaultPin = SS;

/* speed and init speed */
#define SD_SPI_SETTINGS SPISettings(F_CPU, MSBFIRST, SPI_MODE0)
#define SD_SPI_INIT_SETTINGS SPISettings(250000L, MSBFIRST, SPI_MODE0)

//------------------------------------------------------------------------------
// SD operation timeouts
uint8_t const SD_INIT_MAX_CMD0 = 10;
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
/** sd card type v1 */
uint8_t const CARD_TYPE_SDV1 = 0x01;
/** sd card type v2 */
uint8_t const CARD_TYPE_SDV2 = 0x02;
/** sd card type sdhc */
uint8_t const CARD_TYPE_SDHC = 0x03;

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
#ifndef SDCARD_CS_PIN
 SdCard(uint8_t chipSelect = SS) : chipSelectPin(chipSelect) { }
#endif //SDCARD_CS_PIN
  void enableSPI(void);
  bool begin(void);
  bool readBlock(uint32_t block, uint8_t* dst);
  bool writeBlock(uint32_t block, const uint8_t* src);
 private:
#ifndef SDCARD_CS_PIN
  const uint8_t chipSelectPin;
#else
  static constexpr uint8_t chipSelectPin = SDCARD_CS_PIN;
#endif //SDCARD_CS_PIN
  uint8_t cardType;
  uint8_t cardAcmd(uint8_t cmd, uint32_t arg);
  uint8_t cardCommand(uint8_t cmd, uint32_t arg);
  void startSPI(void);
  void stopSPI(void);
  bool waitNotBusy(void);
  uint8_t spiReceive(void) {
    return SPI.transfer(0xff);
  }
};
#endif  // SdCard_h
