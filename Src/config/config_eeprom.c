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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stddef.h>

//#include "../build/build_config.h"

#include "../common/crc.h"

#include "../config/config_eeprom.h"
#include "../config/config_streamer.h"
#include "../config/parameter_group.h"

#include "main.h"
#include "../parameters.h"
#include "../common/utils.h"

// Header for the saved copy.
typedef struct {
    uint8_t format;
} PG_PACKED configHeader_t;


void initEEPROM(void)
{
	//TODO
}

static uint16_t updateCRC(uint16_t crc, const void *data, uint32_t length)
{
	const uint8_t *p = (const uint8_t *)data;
    while(length)
    {
    	length--;
        crc = crc16_ccitt(crc, *(p + length));
    }

    return crc;
}

// Scan the EEPROM config. Returns true if the config is valid.
int isEEPROMContentValid(void)
{
	int i;
	uint8_t buff[4];
	int err;

    config_streamer_t streamer;
    config_streamer_init(&streamer);

    config_streamer_read_start(&streamer, 0x00, EEPROM_PAGE_SIZE);
	configHeader_t header;

	err = config_streamer_read(&streamer, (uint8_t *)&header, sizeof(header));

	if(err != 0)
		return err;

    if (header.format != PARAMETERS_VERSION) {
        return EEPROM_ERROR_VERSION;
    }
    uint16_t crc = updateCRC(0, &header, sizeof(header));

	for(i=0; i<paramsTableLen; i++)
	{
		const setting_t *var = parametersTable[i];

		switch (SETTING_TYPE(var)) {
		case VAR_UINT8:
		case VAR_INT8:
			err = config_streamer_read(&streamer, buff, 1);
			crc = updateCRC(crc, buff, 1);
			break;

		case VAR_UINT16:
		case VAR_INT16:
			err = config_streamer_read(&streamer, buff, 2);
			crc = updateCRC(crc, buff, 2);
			break;

		case VAR_UINT32:
			err = config_streamer_read(&streamer, buff, 4);
			crc = updateCRC(crc, buff, 4);
			break;

		case VAR_FLOAT:
			err = config_streamer_read(&streamer, buff, 4);
			crc = updateCRC(crc, buff, 4);
			break;

		case VAR_STRING:
			// Not supported
			break;
		}
	}

	if(err != 0)
		return err;

    uint16_t checkSum;
    err = config_streamer_read(&streamer, (uint8_t *)&checkSum, sizeof(checkSum));
	if(err != 0)
		return err;

	if(crc != checkSum)
		return EEPROM_ERROR_CRC;

	return 0;
}

// Initialize all PG records from EEPROM.
// This functions processes all PGs sequentially, scanning EEPROM for each one. This is suboptimal,
//   but each PG is loaded/initialized exactly once and in defined order.
int loadEEPROM(void)
{

	uint8_t i;
	int err;

	/* TODO Mayby store all data in arrary then copy to config? */
	err = isEEPROMContentValid();
	if( err != 0)
		return err;

    config_streamer_t streamer;
    config_streamer_init(&streamer);

    config_streamer_read_start(&streamer, 0x00, EEPROM_PAGE_SIZE);
	configHeader_t header;

	err = config_streamer_read(&streamer, (uint8_t *)&header, sizeof(header));

	if(err != 0)
		return err;

    if (header.format != PARAMETERS_VERSION) {
        return EEPROM_ERROR_VERSION;
    }


    uint16_t crc = updateCRC(0, &header, sizeof(header));

	for(i=0; i<paramsTableLen; i++)
	{
		const setting_t *var = parametersTable[i];
		void *ptr = settingGetValuePointer(var);
		int_float_value_t value;
		value.int_value = 0;

		switch (SETTING_TYPE(var)) {
		case VAR_UINT8:
		case VAR_INT8:
			err = config_streamer_read(&streamer, value.b, 1);
			crc = updateCRC(crc, value.b, 1);
			*(int8_t *)ptr = value.int_value;
			break;

		case VAR_UINT16:
		case VAR_INT16:
			err = config_streamer_read(&streamer, value.b, 2);
			crc = updateCRC(crc, value.b, 2);
			*(int16_t *)ptr = value.int_value;
			break;

		case VAR_UINT32:
			err = config_streamer_read(&streamer, value.b, 4);
			crc = updateCRC(crc, value.b, 4);
			*(uint32_t *)ptr = value.uint_value;
			break;

		case VAR_FLOAT:
			err = config_streamer_read(&streamer, value.b, 4);
			crc = updateCRC(crc, value.b, 4);
			*(float *)ptr = (float)value.float_value;
			break;

		case VAR_STRING:
			// Not supported
			break;
		}
	}

	if(err != 0)
		return err;

    uint16_t checkSum;
    err = config_streamer_read(&streamer, (uint8_t *)&checkSum, sizeof(checkSum));
	if(err != 0)
		return err;

	if(crc != checkSum)
		return EEPROM_ERROR_CRC;

	return 0;
}

static int writeSettingsToEEPROM(void)
{
	uint16_t i;
	int err;

    config_streamer_t streamer;
    config_streamer_init(&streamer);

    config_streamer_write_start(&streamer, 0x00, EEPROM_PAGE_SIZE);

    configHeader_t header = {
        .format = PARAMETERS_VERSION,
    };

    err = config_streamer_write(&streamer, (uint8_t *)&header, sizeof(header));

	if(err != 0)
		return err;

    uint16_t crc = updateCRC(0, (uint8_t *)&header, sizeof(header));
    for(i=0; i<paramsTableLen; i++)
    {
    	const setting_t *var = parametersTable[i];

	    switch (SETTING_TYPE(var)) {
	    case VAR_UINT8:
	    case VAR_INT8:
	    	err = config_streamer_write(&streamer, (uint8_t *)var->parameterPointer, 1);
	    	crc = updateCRC(crc, (uint8_t *)var->parameterPointer, 1);
	        break;

	    case VAR_UINT16:
	    case VAR_INT16:
	    	err = config_streamer_write(&streamer, (uint8_t *)var->parameterPointer, 2);
	    	crc = updateCRC(crc, (uint8_t *)var->parameterPointer, 2);
	        break;

	    case VAR_UINT32:
	    	err = config_streamer_write(&streamer, (uint8_t *)var->parameterPointer, 4);
	    	crc = updateCRC(crc, (uint8_t *)var->parameterPointer, 4);
	        break;

	    case VAR_FLOAT:
	    	err = config_streamer_write(&streamer, (uint8_t *)var->parameterPointer, 4);
	    	crc = updateCRC(crc, (uint8_t *)var->parameterPointer, 4);
	        break;

	    case VAR_STRING:
	        // Not supported
	        break;
	    }
    }

	if(err != 0)
		return err;

    // append checksum now
    err = config_streamer_write(&streamer, (uint8_t *)&crc, sizeof(crc));
	if(err != 0)
		return err;

    err = config_streamer_flush(&streamer);
	if(err != 0)
		return err;

    return config_streamer_finish(&streamer);
}

int writeConfigToEEPROM(void)
{
    bool success = false;
    int err=0;
    // write it
    for (int attempt = 0; attempt < 3 && !success; attempt++) {
    	err = writeSettingsToEEPROM();
        if (err==0) {
            success = true;
        }
    }
    if(err!=0)
    	return err;

    err = isEEPROMContentValid();

    return err;

    // Flash write failed - just die now
    //failureMode(FAILURE_FLASH_WRITE_FAILED);
}
