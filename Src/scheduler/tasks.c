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

MotionController MotionX = {
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
		.tmc_addr = 1
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

	if(MotionX.running==FALSE)
	{
		if(flag)
		{
			MotionMoveSteps(&MotionX, 100000, 50000, 10000, 4000);
			flag = 0;
		} else
		{
			MotionMoveSteps(&MotionX, -100000, 50000, 10000, 4000);
			flag = 1;
		}
	}

}

void tasksInitialize()
{
	SchedulerInit(&stsTasks);
	serialPort = usbVcpOpen();

	TaskCreate(&stsTasks, &tLedTask, &led, 255);
	TaskStart(&tLedTask, 500);

	TaskCreate(&stsTasks, &tCore, &Core, 5);
	TaskStart(&tCore, 2000);

	TaskCreate(&stsTasks, &tHandleUSBCommunication, &taskHandleUSBCommunication, 10);
	TaskStart(&tHandleUSBCommunication, 10);

	MotionControllerInitialize(&MotionX);

}

void tasksScheduler()
{
	Scheduler(&stsTasks);
}

