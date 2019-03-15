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

 /**
  * \file
  * Configuration file
  */
#ifndef Fat16Config_h
#define Fat16Config_h
//------------------------------------------------------------------------------
/**
 * Set USE_ACMD41 zero to initialize the card with CMD1.
 * This will allowed limited use of MMC cards.
 */
#define USE_ACMD41 1
/**
 * Set non-zero to allow access to Fat16 internals by cardInfo debug sketch
 */
#define FAT16_DEBUG_SUPPORT 1
#endif  // Fat16Config_h
