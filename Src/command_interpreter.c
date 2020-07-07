/*
 * command_interpreter.c
 *
 *  Created on: 23.06.2020
 *      Author: Neo
 */
#include "command_interpreter.h"
#include "main.h"
#include "common/utils.h"

void getParam(uint8_t *min_payload, uint8_t len_payload)
{

}

void setParam(uint8_t *min_payload, uint8_t len_payload)
{

}

const tHeader CMDArray[] = {
		COMMAND_DEF(0x01, getParam),
		COMMAND_DEF(0x02, setParam)
};

/*
 * Payload structure:
 * Bytes	1		2		3		4		5
 */
void commandInterpreter(tCommandSettings *Node, uint8_t min_id, uint8_t *min_payload, uint8_t len_payload, uint8_t port)
{
	//Decode header and command
	uint8_t header;

	if(len_payload >= 1)
	{
			header = *min_payload++;
			len_payload--;
	} else {
			//payload to small
			//TODO Send some information
			return;
	}

	for(uint8_t i=0; i < ARRAYLEN(CMDArray); i++)
	{
			if(CMDArray[i].header_id == header)
					CMDArray[i].func(min_payload, len_payload);
	}
}

/* Zasada dzia³ania funkcji
 *
 * */
void commandProcess(tCommandSettings *Node)
{
	uint8_t buff[RX_MSG_SIZE];
	uint32_t rxSize;
	uint8_t i;

	//Check if received new data
	rxSize = serialRxBytesWaiting(Node->interface);

	if(rxSize > 0)
	{
		//Get new data
		if(rxSize > RX_MSG_SIZE)
			rxSize = serialRead(Node->interface, &buff[0], RX_MSG_SIZE);
		else
			rxSize = serialRead(Node->interface, &buff[0], rxSize);

		//Check if data consist header
		for(i=0; i<rxSize; i++)
			{
				if(buff[i] == 0xAC)
					return;
			}
	}
}
