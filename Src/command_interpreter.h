/*
 * command_interpreter.h
 *
 *  Created on: 23.06.2020
 *      Author: Neo
 */

#ifndef COMMAND_INTERPRETER_H_
#define COMMAND_INTERPRETER_H_

#include "main.h"
#include "drivers/serial.h"

#define ERROR_CMD 0xA0

#define COMMAND_DEF(header_id, method) \
{ \
    header_id , \
    method \
}

typedef void (*transmitFunc_t)(void *arg, void *data, int count);

typedef struct sCMDInt{
	transmitFunc_t transmit;
	uint8_t min_id;
	uint8_t *min_payload;
	uint8_t len_payload;
	uint8_t port;
} tCMDInt;

typedef struct {
			uint8_t header_id;
	    bool (*func)(tCMDInt *cmd_i);
} tHeader;

void commandInterpreter(tCMDInt *cmd_i);
void commandInit(tCMDInt *cmd_i, transmitFunc_t transmit);

#endif /* COMMAND_INTERPRETER_H_ */
