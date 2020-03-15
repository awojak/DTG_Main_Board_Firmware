/*
 * M24C0x.h
 *
 *  Created on: 14.03.2020
 *      Author: Adrian
 */

#ifndef M24C0X_EEPROM_H_
#define M24C0X_EEPROM_H_

#include "main.h"
#include "eeprom.h"

#define I2C_EEPROM_TIMEOUT 10

void M24C0XInitialize(eeprom_t *e);
uint8_t M24C0XWrite(eeprom_t *instance, uint8_t addr, uint8_t *data, uint8_t length);
uint8_t M24C0XRead(eeprom_t *instance, uint8_t addr, uint8_t *data, uint8_t length);

#endif /* M24C0X_EEPROM_H_ */
