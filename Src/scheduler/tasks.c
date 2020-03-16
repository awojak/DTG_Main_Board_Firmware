/*
 * tasks.c
 *
 *  Created on: 13.03.2020
 *      Author: Adrian
 */
#include "../cli.h"
#include "main.h"
#include "../drivers/motion_controller.h"
#include "usbd_cdc_if.h"
#include "tasks.h"
#include "task_scheduler.h"
#include "../drivers/serial.h"
#include "../drivers/usbd_cdc_vcp.h"
#include "../drivers/eeprom.h"
#include "../drivers/M24C0x.h"
#include "../parameters.h"

extern I2C_HandleTypeDef hi2c3;
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim10;
extern USBD_HandleTypeDef hUsbDeviceFS;

serialPort_t *serialPort = NULL;

SchedulerTasks stsTasks;
Task tHandleUSBCommunication;

Task tLedTask, tSendPos;
Task tCore;

char text[12] = {0};
uint8_t flag = 1;

MotionController MotionY = {
		.back_down_limit_gpio_port = 0,
		.back_down_limit_pin = 0,
		.front_up_limit_gpio_port = 0,
		.front_up_limit_pin = 0,
		.dir_gpio_port = Y_DIR_GPIO_Port,
		.dir_pin = Y_DIR_Pin,
		.step_gpio_port = Y_STEP_GPIO_Port,
		.step_pin = Y_STEP_Pin,
		.timer = &htim10,
		.uart = &huart2,
		.tmc_addr = 1,
		.rampPrint.speed = PARAMETER_PRINT_SPEED_Y,
		.rampPrint.accel = PARAMETER_PRINT_ACCEL_Y,
		.rampPrint.decel = PARAMETER_PRINT_DECEL_Y,
		.rampJog.speed = PARAMETER_JOG_SPEED_Y,
		.rampJog.accel = PARAMETER_JOG_ACCEL_Y,
		.rampJog.decel = PARAMETER_JOG_DECEL_Y,
		.rampMove.speed = PARAMETER_MOVE_SPEED_Y,
		.rampMove.accel = PARAMETER_MOVE_ACCEL_Y,
		.rampMove.decel = PARAMETER_MOVE_DECEL_Y
};

eeprom_t eeprom = {
	    .hi2c = &hi2c3,
	    .devAddr = EEPROM_ADDR,
		.lock_gpio_port = EEPROM_P_GPIO_Port,
		.lock_pin = EEPROM_P_Pin,
		.lock_pin_state = GPIO_PIN_SET

};


static void taskHandleUSBCommunication()
{
    // in cli mode, all serial stuff goes to here. enter cli mode by sending #
    if (cliMode) {
        cliProcess();
    } else
    {
		//Check if enter to cli
		if(serialPort!=NULL)
		{
			while(serialRxBytesWaiting(serialPort))
			{
				uint8_t c = serialRead(serialPort);
				if (c == '#') {
					cliEnter(serialPort);
				}
			}
		}
    }

    // Allow MSP processing even if in CLI mode
    //mspSerialProcess(ARMING_FLAG(ARMED) ? MSP_SKIP_NON_MSP_DATA : MSP_EVALUATE_NON_MSP_DATA, mspFcProcessCommand);
}

/**
 *  Led task
 */
void led()
{
	HAL_GPIO_TogglePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin);
}

/**
 * USB Send task
 */
void usbSend()
{
	/* CDC structure where we check if the USB is transmitting */
	USBD_CDC_HandleTypeDef  *hcdc;
	/* if USB ready and sent previous data start transmitting*/
	if(hUsbDeviceFS.dev_state ==  USBD_STATE_CONFIGURED && (hcdc = hUsbDeviceFS.pClassData) != 0 && hcdc->TxState == 0)
	{
	  /* send data over usb, we have 1000 16bit samples which is 2000bytes */
	  //TODO Use usb to send data instead UART
	  //CDC_Transmit_FS((uint8_t *)aDataP, 2000);
	  /* after finish transmit we can send more */
	}
}

void Core()
{
	uint8_t data[10];
	uint8_t temp[10];
	uint8_t i;
	if(MotionY.running==FALSE)
	{
		if(flag)
		{
			MotionJogSteps(&MotionY, 100000);
			flag = 0;
		} else
		{
			MotionJogSteps(&MotionY, -100000);
			flag = 1;
		}
	}

	for(i=0; i<10; i++)
		data[i] = i+1;
	eepromWrite(&eeprom, 0x00, data, 10);
	//Should be delay here
	HAL_Delay(5);
	eepromRead(&eeprom, 0x00, temp, 10);

	if(data[0]==temp[0])
		temp[0] = 10;
}

void tasksInitialize()
{
	SchedulerInit(&stsTasks);

	M24C0XInitialize(&eeprom);
	serialPort = usbVcpOpen();

	TaskCreate(&stsTasks, &tLedTask, &led, 255);
	TaskStart(&tLedTask, 500);

	TaskCreate(&stsTasks, &tCore, &Core, 5);
	TaskStart(&tCore, 2000);

	TaskCreate(&stsTasks, &tHandleUSBCommunication, &taskHandleUSBCommunication, 10);
	TaskStart(&tHandleUSBCommunication, 10);

	MotionControllerInitialize(&MotionY);

}

void tasksScheduler()
{
	Scheduler(&stsTasks);
}

