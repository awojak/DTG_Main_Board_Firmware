/*
 * eeprom_memory.c
 *
 *  Created on: 14.03.2020
 *      Author: Adrian
 */
#include "eeprom.h"

eeprom_t eeprom = {
	    .hi2c = &hi2c3,
	    .devAddr = EEPROM_ADDR,
		.lock_gpio_port = EEPROM_P_GPIO_Port,
		.lock_pin = EEPROM_P_Pin,
		.lock_pin_state = GPIO_PIN_SET

};


uint8_t eepromWrite(eeprom_t *instance, uint8_t addr, uint8_t *data, uint8_t length)
{
    return instance->vTable->eepromWrite(instance, addr, data, length);
}

uint8_t eepromRead(eeprom_t *instance, uint8_t addr, uint8_t *data, uint8_t length)
{
    return instance->vTable->eepromRead(instance, addr, data, length);
}

void eepromUnlock(eeprom_t *instance)
{
	instance->vTable->eepromUnlock(instance);
}

void eepromLock(eeprom_t *instance)
{
	instance->vTable->eepromLock(instance);
}
