/*
 * M24C0x.c
 *
 *  Created on: 14.03.2020
 *      Author: Adrians
 */
#include "M24C0x.h"

static void lockEEPROM(eeprom_t *instance)
{
	HAL_GPIO_WritePin(instance->lock_gpio_port, instance->lock_pin, instance->lock_pin_state);
}

static void unclockEEPROM(eeprom_t *instance)
{
	GPIO_PinState state;
	if(instance->lock_pin_state)
		state = 0;
	else
		state = 1;

	HAL_GPIO_WritePin(instance->lock_gpio_port, instance->lock_pin, state);
}

uint8_t M24C0XWrite(eeprom_t *instance, uint8_t addr, uint8_t *data, uint8_t length)
{
	uint8_t repeat = 0;
	HAL_StatusTypeDef status;

	do {
		status = HAL_I2C_Mem_Write(instance->hi2c, I2C_SHIFTED_ADDR(instance->devAddr), addr, I2C_MEMADD_SIZE_8BIT, data, length, I2C_EEPROM_TIMEOUT);
		HAL_Delay(5);
		repeat++;
	} while((status != HAL_OK) && (repeat < 4));

	return status;
}

uint8_t M24C0XRead(eeprom_t *instance, uint8_t addr, uint8_t *data, uint8_t length)
{
	uint8_t repeat = 0;
	HAL_StatusTypeDef status;

	do {
		status = HAL_I2C_Mem_Read(instance->hi2c, I2C_SHIFTED_ADDR(instance->devAddr), addr, I2C_MEMADD_SIZE_8BIT, data, length, I2C_EEPROM_TIMEOUT);
		HAL_Delay(5);
		repeat++;
	} while((status != HAL_OK) && (repeat < 4));

	return status;
}

static const struct eppromVTable M24C0XVTable[] = {
    {
        .eepromRead = M24C0XRead,
        .eepromWrite = M24C0XWrite,
		.eepromLock = lockEEPROM,
		.eepromUnlock = unclockEEPROM
    }
};

void M24C0XInitialize(eeprom_t *e)
{
	if(e!=NULL)
		e->vTable = M24C0XVTable;
}
