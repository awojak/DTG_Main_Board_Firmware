/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define EEPROM_ERROR_CRC -7
#define EEPROM_ERROR_VERSION -6
#define EEPROM_ERROR_NO_MEMORY -4
#define EEPROM_ERROR_END_MEMORY -5

int isEEPROMContentValid(void);
int loadEEPROM(void);
int writeConfigToEEPROM(void);
uint16_t getEEPROMConfigSize(void);
