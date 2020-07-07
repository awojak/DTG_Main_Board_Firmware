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

typedef struct {
			uint8_t header_id;
	    void (*func)(uint8_t *min_payload, uint8_t len_payload);
} tHeader;

#define COMMAND_DEF(header_id, method) \
{ \
    header_id , \
    method \
}

/* Message buffers max size */
/* TODO Always adjust to your application */
#define RX_MSG_SIZE 24
#define TX_MSG_SIZE 24

typedef struct sCommandSettings {
	serialPort_t *interface;
	uint8_t rx_data[RX_MSG_SIZE];
	uint8_t rx_index;
	uint8_t tx_data[TX_MSG_SIZE];
	uint8_t tx_index;

	//flags
	uint8_t frame_complete; // 0 - no wait for more data, 1 - yes
	/* Sometimes interface is busy or need more time to answer, check if replied */
	uint8_t replied; // 0 - now try answer again, 1- yes

	/* Store time of received first part of frame, useful for timeout */
	int32_t start_time;

} tCommandSettings;

void commandInterpreter(tCommandSettings *Node, uint8_t min_id, uint8_t *min_payload, uint8_t len_payload, uint8_t port);

#endif /* COMMAND_INTERPRETER_H_ */
