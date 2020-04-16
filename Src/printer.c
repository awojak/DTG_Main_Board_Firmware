/*
 * printer.c
 *
 *  Created on: 15.04.2020
 *      Author: Adrian Wojak
 */

#include "printer.h"
#include "drivers/motion_controller.h"

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
	p->MotionZ->not_safe = 0;

	//TODO don't do it in this way, write function to check safety status
	if(p->photo_barier_state == 0)
		p->MotionY->not_safe = 0;

	//TODO wyœlij komunikat gdzieœ dalej o zmianie statusuS
}

void PrinterInitialize(tPrinter *p)
{
	if(!p->service_mode)
	{
		//Check if already Initialized
		if(isPrinterInitialized(p))
			return;

		//Check if emergency active
		if(isEmergencyActive(p))
			return;
	}

	//TODO Before start process

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

void PrinterProcess(tPrinter *p)
{
	// Check emergency
	if(p->emergency_state == 1)
	{
		//check if button inactive
		if((p->emergency_gpio_port->IDR & p->emergency_pin) != (uint32_t)p->emergency_pin_active_level)
		{
			//Exit emergency state
			DeactivateEmergency(p);
		} else {
			//Exit
			return;
		}
	} else {
		//Additional check instead IRQ, because IRQ is just for edge
		if((p->emergency_gpio_port->IDR & p->emergency_pin) == (uint32_t)p->emergency_pin_active_level)
		{
			EmergencyIRQ(p);
		}
	}

	//Check Photo Barier State
	if(p->photo_barier_state == 1)
	{
		//Check if inactive
		if((p->photo_barier_gpio_port->IDR & p->photo_barier_pin) != (uint32_t)p->photo_barier_active_level)
		{
			p->photo_barier_state = 0;
			p->MotionY->not_safe = 0;
		}
	} else {
		//Additional check instead IRQ, because IRQ is just for edge
		if((p->photo_barier_gpio_port->IDR & p->photo_barier_pin) == (uint32_t)p->photo_barier_active_level)
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
