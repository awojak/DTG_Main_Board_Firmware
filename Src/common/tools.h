#ifndef _TOOLS_H_
#define _TOOLS_H_

#include "stdint.h"

int IntToChar(int number, char *text);
unsigned char BCDToNumber(unsigned char n);
unsigned long Sqrt(unsigned long x);
unsigned int min(unsigned int x, unsigned int y);
void calcCRC(unsigned char* datagram, unsigned char datagramLength);
void float2Bytes(float val, uint8_t *byte_array);
#endif
