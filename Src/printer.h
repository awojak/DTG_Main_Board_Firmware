/*
 * printer.h
 *
 *  Created on: 15.04.2020
 *      Author: Adrian Wojak
 */

#ifndef PRINTER_H_
#define PRINTER_H_

#include "drivers/motion_controller.h"
#include "scheduler/semaphore.h"

typedef enum ePrinterState { IDLE = 0, PRINT, PRINTING, FINISH } PrinterState;
/* Structure contain all data of the printer */
typedef struct sPrinter {

	// Emergency, GPIO port and pin, should be configured as input with IRQ
	GPIO_TypeDef *emergency_gpio_port;
	uint16_t emergency_pin;
	// Default active level for emergency 0- low, 1- high
	uint8_t emergency_pin_active_level;

	// PE Signal, GPIO port and pin, should be configured as output
	GPIO_TypeDef *pe_signal_gpio_port;
	uint16_t pe_signal_pin;
	// Paper detect pe_signal level 0- low or 1- high
	uint8_t pe_signal_active_level;

	// Photo barier, GPIO port and pin, should be configured as input with IRQ
	GPIO_TypeDef *photo_barier_gpio_port;
	uint16_t photo_barier_pin;
	// Default active level for photo barier 0- low, 1- high
	uint8_t photo_barier_active_level;

	//encoder TIMER instance
	TIM_TypeDef *enc_timer;

	//Variables for system
	//Epson encoder position divide by the value
	float stepper_factor;
	//Epson encoder pulses count
	int encoder_count;
	//Start position for printing
	int prtint_start_position;
	//PE Sensor singal activation limit during priting
	int pe_lower_limit;
	int pe_upper_limit;
	//Eject trigger position, detect finish printing
	int eject_trigger_position;
	//Eject plate position
	int eject_position;

	// states
	uint8_t pe_state;
	uint8_t initilize_state;
	Semaphore sem_init;
	uint8_t emergency_state;
	uint8_t photo_barier_state;
	PrinterState printer_state;

	// Motion Controllers
	MotionController *MotionY;
	MotionController *MotionZ;

	// Timeouts
	uint32_t timeout_motion_z;
	uint32_t timeout_motion_y;

	uint8_t service_mode;

} tPrinter;

void PrinterProcess(tPrinter *p);
void EmergencyIRQ(tPrinter *p);
void PhotoBarierIRQ(tPrinter *p);
void PrinterInitialize(tPrinter *p);
uint8_t isEmergencyActive(tPrinter *p);
uint8_t isPrinterInitialized(tPrinter *p);

#endif /* PRINTER_H_ */
