/*
 * printer.c
 *
 *  Created on: 15.04.2020
 *      Author: Adrian Wojak
 */

#include "printer.h"
#include "drivers/motion_controller.h"

/* Paper detect */
inline void activePE(tPrinter *p)
{
	//Normalnie musimy ustawic pin w stanie niskim
	GPIO_PinState state;
	if(p->pe_signal_active_level)
		state = GPIO_PIN_SET;
	else
		state = GPIO_PIN_RESET;

	HAL_GPIO_WritePin(p->pe_signal_gpio_port, p->pe_signal_pin, state);
	p->pe_state = 1;
}

/* No paper detect */
inline void deactivePE(tPrinter *p)
{
	GPIO_PinState state;
	if(p->pe_signal_active_level)
		state = GPIO_PIN_RESET;
	else
		state = GPIO_PIN_SET;

	HAL_GPIO_WritePin(p->pe_signal_gpio_port, p->pe_signal_pin, state);
	p->pe_state = 0;
}

static void ActiveEmergency(tPrinter *p)
{
	//if already active, exit function
	if(p->emergency_state == p->emergency_pin_active_level)
		return;

	p->emergency_state = 1;
	p->initilize_state = 0;

	//Disable Motors
	MotionDisable(p->MotionY);
	MotionDisable(p->MotionZ);

	//Deactive Semaphores
	SemDeactivate(&p->sem_init);

	//TODO wyœlij komunikat gdzieœ dalej o zmianie statusu

}

static void DeactivateEmergency(tPrinter *p)
{
	//if already inactive, exit function
	if(p->emergency_state != p->emergency_pin_active_level)
		return;

	p->emergency_state = 0;
	//p->MotionZ->not_safe = 0;

	//TODO don't do it in this way, write function to check safety status
	//if(p->photo_barier_state == 0)
		//p->MotionY->not_safe = 0;

	//TODO wyœlij komunikat gdzieœ dalej o zmianie statusuS
}

void PrinterInitialize(tPrinter *p)
{
	if(!p->service_mode)
	{
		//Check if already Initialized
		//if(isPrinterInitialized(p))
		//	return;

		//Check if emergency active
		if(isEmergencyActive(p))
			return;
	}

	//TODO Before start process

	//PE Signal - no paper
	deactivePE(p);

	//Start Initialize
	SemActivate(&p->sem_init, 0);
}

/**
 *	This function disable Motion Y and should be execute in IRQ to be fast as possible
 */
inline void PhotoBarierIRQ(tPrinter *p)
{
	//Active not safe mode for Motion Y
	p->photo_barier_state = 1;
	p->MotionY->not_safe = 1;
	p->MotionZ->not_safe = 1;
}

/**
 *	This function enabled emergency state and should be execute in IRQ to be fast as possible
 */
void EmergencyIRQ(tPrinter *p)
{
	// If IRQ is only for active state, remove IF
	//if((p->emergency_gpio_port->IDR & p->emergency_pin) == (uint32_t)p->emergency_pin_active_level)
	//{
		//active emergnecy mode
		ActiveEmergency(p);
	//}
}

/* Every 1ms, main printing loop, should be disable during non printing */
void PrinterPrintingProcess(tPrinter *p)
{
	int encoder_new_value;

	int stepper_position;
	/* Printer should be initialized and homed before execute this process */
	/* Function don't check states to be fast ASAP */
	/* Table should be at start printing position */

	switch(p->printer_state)
	{
		case IDLE:
		break;

		case PRINT:

			//Reset timer counter to printer start position
			p->encoder_count = 0;//p->print_start_position * p->stepper_factor;
			p->enc_timer->CNT = 0;//p->encoder_count;


			//deactive PE
			deactivePE(p);

			//go to start position
			MotionMovePos(p->MotionY, p->print_start_position);

			if(!(p->MotionY->running))
			{
				//Wait for stop running
				//go to printing state
				p->printer_state = PRINTING;
				//TODO implement timeout
			}
		break;

		case PRINTING:

			//Read EPSON encoder value
			encoder_new_value = p->enc_timer->CNT;

			//Check if encoder wheel has moved
			if((encoder_new_value != p->encoder_count)&&(!(p->MotionY->running)))
			{
				stepper_position = encoder_new_value / p->stepper_factor;

				//Go to calculated stepper position !
				//TODO Finish MotionMovePrinting function!
				MotionMovePrinting(p->MotionY, p->print_start_position + stepper_position);
			}

		    // At XXX encoder trigger the PE signal
		   if ((encoder_new_value > p->pe_lower_limit) && (encoder_new_value  < p->pe_upper_limit))
		   {
			   //TODO be carefully to not skip if EPSON encoder increase to much
			   activePE(p);
		   }

		   if(encoder_new_value > p->eject_trigger_position)
		   {
			   //Printing finished
			   deactivePE(p);
			   p->printer_state = FINISH;
		   }

		   //Save encoder pos for next calculation
		   p->encoder_count = encoder_new_value;
		break;

		case FINISH:
			//Go to Eject position
			//TODO check if move is possible, previous printing movement should be stopped
			MotionMovePos(p->MotionY, p->eject_position);
			//Go to IDLE or deactive task
			p->printer_state = IDLE;
		break;
	}
}


void PrinterProcess(tPrinter *p)
{
	// Check emergency
	if(p->emergency_state == 1)
	{
		//check if button inactive
		if(HAL_GPIO_ReadPin(p->emergency_gpio_port, p->emergency_pin) != (uint32_t)p->emergency_pin_active_level)
		{
			//Exit emergency state
			DeactivateEmergency(p);
		} else {
			//Exit
			return;
		}
	} else {
		//Additional check instead IRQ, because IRQ is just for edge
		if(HAL_GPIO_ReadPin(p->emergency_gpio_port, p->emergency_pin) == (uint32_t)p->emergency_pin_active_level)
		{
			EmergencyIRQ(p);
		}
	}

	//Check Photo Barier State
	if(p->photo_barier_state == 1)
	{
		//Check if inactive
		if(HAL_GPIO_ReadPin(p->photo_barier_gpio_port, p->photo_barier_pin) != (uint32_t)p->photo_barier_active_level)
		{
			p->photo_barier_state = 0;
			p->MotionY->not_safe = 0;
			p->MotionZ->not_safe = 0;
		}
	} else {
		//Additional check instead IRQ, because IRQ is just for edge
		if(HAL_GPIO_ReadPin(p->photo_barier_gpio_port, p->photo_barier_pin) == (uint32_t)p->photo_barier_active_level)
		{
			PhotoBarierIRQ(p);
		}
	}

	//If emergency active, not go here

	//Initialize Semaphore check
	if(SemCheck(&p->sem_init))
	{
		switch(SemGetProcess(&p->sem_init))
		{
			case 0:
				//TODO Check all inputs, limits, sensors etc
				SemSetProcess(&p->sem_init,1);
			break;

			case 1:
				//Enable Motion Controllers
				MotionEnable(p->MotionZ);
				MotionEnable(p->MotionY);
				SemSetProcess(&p->sem_init,2);
			break;

			case 2:
				//Home Axis Z
				if(!isHomed(p->MotionZ))
					MotionHome(p->MotionZ);

				//Set time for timeout
				SemSetTime(&p->sem_init, GetTicks());
				SemSetProcess(&p->sem_init,3);
			break;

			case 3:
				//Check Axis Z Homing status
				if(isHomed(p->MotionZ))
					SemSetProcess(&p->sem_init,4);
				else {
					//check timeout
					if(GetTicks()-SemGetTime(&p->sem_init) >= p->timeout_motion_z)
					{
						SemDeactivate(&p->sem_init);
						//TODO Homing failed emit error
					}
				}
			break;
			case 4:
				//Home Axis Y
				if(!isHomed(p->MotionY))
					MotionHome(p->MotionY);

				//Set time for timeout
				SemSetTime(&p->sem_init, GetTicks());
				SemSetProcess(&p->sem_init,5);
			break;

			case 5:
				//Check Axis Y Homing status
				if(isHomed(p->MotionY))
					SemSetProcess(&p->sem_init,6);
				else {
					//check timeout
					if(GetTicks()-SemGetTime(&p->sem_init) >= p->timeout_motion_y)
					{
						SemDeactivate(&p->sem_init);
						//TODO Homing failed emit error
					}
				}
			break;

			case 6:
				//Successfully initialized
				p->initilize_state = 1;
				SemDeactivate(&p->sem_init);
			break;
			default:
			break;
				//Error!
		}
	}

	//Next
}

uint8_t isEmergencyActive(tPrinter *p)
{
	return p->emergency_state;
}

uint8_t isPrinterInitialized(tPrinter *p)
{
	return p->initilize_state;
}
