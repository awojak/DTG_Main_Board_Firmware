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

#include <string.h>

#include "main.h"

#include "config_streamer.h"
#include "config_eeprom.h"
#include "../drivers/eeprom.h"

void config_streamer_init(config_streamer_t *c)
{
    memset(c, 0, sizeof(*c));
}

void config_streamer_read_start(config_streamer_t *c, uint16_t base, int size)
{
    // base must start at FLASH_PAGE_SIZE boundary
    c->address = base;
    c->size = size;
    c->err = 0;
}

void config_streamer_write_start(config_streamer_t *c, uint16_t base, int size)
{
	config_streamer_read_start(c, base, size);
    if (!c->unlocked) {
    	eepromUnlock(&eeprom);
    	c->unlocked = true;
    }
}

static int read_word(config_streamer_t *c)
{
    if (c->err != 0) {
        return c->err;
    }

    if((c->address + 4) >= EEPROM_PAGE_SIZE)
    {
    	//EEPROM Page is finished
    	return EEPROM_ERROR_END_MEMORY;
    } else
    {
		const HAL_StatusTypeDef status = eepromRead(&eeprom, c->address, c->buffer.b, 4);
		if (status != HAL_OK) {
			return status;
		}
    }

    c->address += 4;

    return 0;
}

static int write_word(config_streamer_t *c, uint32_t value)
{
    if (c->err != 0) {
        return c->err;
    }

    if((c->address + sizeof(value)) >= EEPROM_PAGE_SIZE)
    {
    	//Not enough space in EEPROM
    	return EEPROM_ERROR_NO_MEMORY;
    } else
    {
		const HAL_StatusTypeDef status = eepromWrite(&eeprom, c->address, (uint8_t *) &value, sizeof(value));
		if (status != HAL_OK) {
			return status;
		}
    }

    c->address += sizeof(value);

    return 0;
}

int config_streamer_read(config_streamer_t *c, uint8_t *p, uint32_t size)
{
	int i;
    for (i=0; i<size; i++) {

        if (c->at == 0) {
        	//read one word and store in buffer
            c->err = read_word(c);
            c->at = 4;
        }
        //read one byte
        *(p+i) = c->buffer.b[4 - c->at];
        c->at--;
    }
    return c->err;
}

int config_streamer_write(config_streamer_t *c, const uint8_t *p, uint32_t size)
{
    for (const uint8_t *pat = p; pat != (uint8_t*)p + size; pat++) {
        c->buffer.b[c->at++] = *pat;

        if (c->at == sizeof(c->buffer)) {
            c->err = write_word(c, c->buffer.w);
            c->at = 0;
        }
    }
    return c->err;
}

int config_streamer_status(config_streamer_t *c)
{
    return c->err;
}

int config_streamer_flush(config_streamer_t *c)
{
    if (c->at != 0) {
        memset(c->buffer.b + c->at, 0, sizeof(c->buffer) - c->at);
        c->err = write_word(c, c->buffer.w);
        c->at = 0;
    }
    return c-> err;
}

int config_streamer_finish(config_streamer_t *c)
{
    if (c->unlocked) {
    	eepromLock(&eeprom);
        c->unlocked = false;
    }
    return c->err;
}
