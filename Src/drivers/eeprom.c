/*
 * eeprom_memory.c
 *
 *  Created on: 14.03.2020
 *      Author: Adrian
 */
#include "eeprom.h"

uint8_t eepromWrite(eeprom_t *instance, uint8_t addr, uint8_t *data, uint8_t length)
{
    return instance->vTable->eepromWrite(instance, addr, data, length);
}

uint8_t eepromRead(eeprom_t *instance, uint8_t addr, uint8_t *data, uint8_t length)
{
    return instance->vTable->eepromRead(instance, addr, data, length);
}
