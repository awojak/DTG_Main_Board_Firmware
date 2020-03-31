/*
 * eeprom_memory.h
 *
 *  Created on: 14.03.2020
 *      Author: Adrian
 */

#ifndef DRIVERS_EEPROM_H_
#define DRIVERS_EEPROM_H_

#include "main.h"

typedef struct eeprom_s {
    const struct eppromVTable *vTable;

    I2C_HandleTypeDef *hi2c;
    uint8_t devAddr;
    /* For write protection */
	GPIO_TypeDef *lock_gpio_port;
	uint16_t lock_pin;

	/* Default pin state for lock EEPROM */
	GPIO_PinState lock_pin_state;

	uint8_t isConnected;

} eeprom_t;

extern eeprom_t eeprom;

struct eppromVTable {
    uint8_t (*eepromRead)(eeprom_t *instance, uint8_t addr, uint8_t *data, uint8_t length);
    uint8_t (*eepromWrite)(eeprom_t *instance, uint8_t addr, uint8_t *data, uint8_t length);
    void (*eepromUnlock)(eeprom_t *instance);
    void (*eepromLock)(eeprom_t *instance);
};

uint8_t eepromWrite(eeprom_t *instance, uint8_t addr, uint8_t *data, uint8_t length);
uint8_t eepromRead(eeprom_t *instance, uint8_t addr, uint8_t *data, uint8_t length);
void eepromUnlock(eeprom_t *instance);
void eepromLock(eeprom_t *instance);

#endif /* DRIVERS_EEPROM_H_ */
