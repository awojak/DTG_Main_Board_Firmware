/*
 * cli.h
 *
 *  Created on: 27.02.2020
 *      Author: Neo
 */

#ifndef CLI_H_
#define CLI_H_

#include "main.h"

extern uint8_t cliMode;

typedef struct {
    const char *name;
#ifndef SKIP_CLI_COMMAND_HELP
    const char *description;
    const char *args;
#endif
    void (*func)(char *cmdline);
} clicmd_t;

//struct serialConfig_s;
//void cliInit(const struct serialConfig_s *serialConfig);
void cliProcess(void);
struct serialPort_s;
void cliEnter(struct serialPort_s *serialPort);

#endif /* CLI_H_ */
