/*
 * command_interpreter.c
 *
 *  Created on: 23.06.2020
 *      Author: Neo
 */
#include "command_interpreter.h"

/* Zasada dzia³ania funkcji
 *
 * */
void commandProcess(tNodeSettings *Node)
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
