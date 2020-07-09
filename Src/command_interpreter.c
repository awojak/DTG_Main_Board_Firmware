/*
 * command_interpreter.c
 *
 *  Created on: 23.06.2020
 *      Author: Neo
 */
#include "command_interpreter.h"
#include "main.h"
#include "common/utils.h"
#include <stdbool.h>
#include "parameters.h"
#include "common/tools.h"

/*
 * Function use uint8_t type for parameters index
 */
#if paramTableLen > 256
#error "Function doesn't support parameters length more then 255"
#endif

void commandInit(tCMDInt *cmd_i, transmitFunc_t transmit)
{
	cmd_i->transmit = transmit;
}

bool getParamValue(tCMDInt *cmd_i)
{
	uint16_t id;
	uint8_t buffer[7];
	uint8_t s;
	uint8_t *buf = buffer;

	//Check payload size
	if(cmd_i->len_payload < 2)
	{
		//Payload too small
		return false;
	}

	//Get payload id
	id = (*cmd_i->min_payload++) & 0xFF;
	id = ((*cmd_i->min_payload++) << 8) & 0xFF00;
	//cmd_i->len_payload -= 2;

	//looking for parameter
	if(id <= paramsTableLen)
	{
		*(buf++) = 0x01; //Header get data
		*(buf++) = id & 0xFF;
		*(buf++) = (id >> 8) & 0xFF;

		const setting_t *p;
		p = settingGet(id);
		s = settingGetValueSize(p);
		//Not working with String Type
		void *v = settingGetValuePointer(p);
		switch (SETTING_TYPE(p)) {
			case VAR_UINT8:
				*(buf++) = *((uint8_t *)v) & 0xFF;
				break;
			case VAR_INT8:
				*(buf++) = (uint8_t)*((int8_t *)v) & 0xFF;
				break;
			case VAR_UINT16:
				*(buf++) = *((uint16_t *)v) & 0xFF;
				*(buf++) = (*((uint16_t *)v) >> 8) & 0xFF;
				break;
			case VAR_INT16:
				*(buf++) = *((int16_t *)v) & 0xFF;
				*(buf++) = (*((int16_t *)v) >> 8) & 0xFF;
				break;
			case VAR_UINT32:
				*(buf++) = *((uint32_t *)v) & 0xFF;
				*(buf++) = (*((uint32_t *)v) >> 8) & 0xFF;
				*(buf++) = (*((uint32_t *)v) >> 16) & 0xFF;
				*(buf++) = (*((uint32_t *)v) >> 24) & 0xFF;
				break;
			case VAR_FLOAT:
				//TODO Check if better send as text
				float2Bytes(*((float *)v), buf);
				break;
			case VAR_STRING:
				break;
		}

		//Transmit answer
		cmd_i->transmit(cmd_i, buf, s+3);

		/*
		switch(s)
		{
		case 4:
		{
			uint32_t *v = (uint32_t *)settingGetValuePointer(p);
			buf[2] = ((*v) >> 16);
			buf[3] = ((*v) >> 24);
		}

		case 2:
		{
			uint16_t *v = (uint16_t *)settingGetValuePointer(p);
			buf[1] = ((*v) >> 8);
		}

		case 1:
		{
			uint8_t *v = (uint8_t *)settingGetValuePointer(p);
			buf[0] = *v;
		}
		};
		*/

	}
	else
		return false;

	return true;
}

bool setParamValue(tCMDInt *cmd_i)
{
	uint16_t id;

	//Check payload size
	if(cmd_i->len_payload < 2)
	{
		//Payload too small
		return false;
	}

	//Get payload id
	id = (*cmd_i->min_payload++) & 0xFF;
	id = ((*cmd_i->min_payload++) << 8) & 0xFF00;
	cmd_i->len_payload -= 2;

	//looking for parameter
	if(id <= paramsTableLen)
	{
		const setting_t *p;
		p = settingGet(id);
		//Not working with String Type
		void *v = settingGetValuePointer(p);
		switch (SETTING_TYPE(p)) {
			case VAR_UINT8:
				//TODO
				break;
			case VAR_INT8:
				//TODO
				break;
			case VAR_UINT16:
				//TODO
				break;
			case VAR_INT16:
				//TODO
				break;
			case VAR_UINT32:
				//TODO
				break;
			case VAR_FLOAT:
				//TODO
				break;
			case VAR_STRING:
				break;
		}
	}
	else
		return false;

	return true;
}

bool systemMethods(tCMDInt *cmd_i)
{
	return false;
}

const tHeader CMDArray[] = {
		COMMAND_DEF(0x01, getParamValue),
		COMMAND_DEF(0x02, setParamValue),
		COMMAND_DEF(0x10, systemMethods)
};

/*
 * Payload structure:
 * Bytes	1		2		3		4		5
 */
void commandInterpreter(tCMDInt *cmd_i)
{
	//Decode header and command
	uint8_t header;
	bool result;

	if(cmd_i->len_payload >= 1)
	{
			header = *cmd_i->min_payload++;
			cmd_i->len_payload--;
	} else {
			//payload to small
			//TODO Send some information
			return;
	}

	for(uint8_t i=0; i < ARRAYLEN(CMDArray); i++)
	{
			if(CMDArray[i].header_id == header)
			{
				result = CMDArray[i].func(cmd_i);
				// If result false send
				if(!result)
					cmd_i->transmit(cmd_i, (uint8_t *)ERROR_CMD, 1);

				break;
			}
	}
}

/* Zasada dzia³ania funkcji
 *
 * */
void commandProcess(tCMDInt *cmd_i)
{

}
