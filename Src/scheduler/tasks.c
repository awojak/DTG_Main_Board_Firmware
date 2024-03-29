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
#include "../printer.h"
#include "../min.h"
#include "../UART_DMA.h"
#include "../command_interpreter.h"

extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim9;
extern USBD_HandleTypeDef hUsbDeviceFS;

extern UARTDMA_HandleTypeDef huartdma;

serialPort_t *serialPort = NULL;

struct min_context min_pc;
struct min_context min_hmi;

SchedulerTasks stsTasks;
Task tHandleUSBCommunication;
Task tHandleUART8;

Task tPrinterProcess, tMotionProcess;
Task tLedTask, tSendPos;
Task tCore;

tCMDInt CMD;
char text[12] = {0};
uint8_t flag = 1;
uint8_t minMode = 1;
uint8_t enableCli = 1;

MotionController MotionY = {
		.back_down_limit_gpio_port = LIMIT_Y_BACK_GPIO_Port,
		.back_down_limit_pin = LIMIT_Y_BACK_Pin,
		.back_down_limit_active_level = 0,
		.front_up_limit_gpio_port = LIMIT_Y_FRONT_GPIO_Port,
		.front_up_limit_pin = LIMIT_Y_FRONT_Pin,
		.front_up_limit_active_level = 0,
		.enable_gpio_port = Y_ENABLE_GPIO_Port,
		.enable_pin = Y_ENABLE_Pin,
		.enable_active_level = 0,
		.enable_state = 0,
		.dir_gpio_port = Y_DIR_GPIO_Port,
		.dir_pin = Y_DIR_Pin,
		.step_gpio_port = Y_STEP_GPIO_Port,
		.step_pin = Y_STEP_Pin,
		.forward_dir = CW,
		.timer = &htim10,
		.uart = &huart2,
		.tmc_addr = 1,
		.rampHome.speed = PARAMETER_HOME_SPEED_Y,
		.rampHome.accel = PARAMETER_HOME_ACCEL_Y,
		.rampHome.decel = PARAMETER_HOME_DECEL_Y,
		.rampPrint.speed = PARAMETER_PRINT_SPEED_Y,
		.rampPrint.accel = PARAMETER_PRINT_ACCEL_Y,
		.rampPrint.decel = PARAMETER_PRINT_DECEL_Y,
		.rampJog.speed = PARAMETER_JOG_SPEED_Y,
		.rampJog.accel = PARAMETER_JOG_ACCEL_Y,
		.rampJog.decel = PARAMETER_JOG_DECEL_Y,
		.rampMove.speed = PARAMETER_MOVE_SPEED_Y,
		.rampMove.accel = PARAMETER_MOVE_ACCEL_Y,
		.rampMove.decel = PARAMETER_MOVE_DECEL_Y,
		.home_timeout = 20000,
		.standby_position = 50000,
		.not_safe = 0,
		.safe_dir = 1
};

MotionController MotionZ = {
		.back_down_limit_gpio_port = LIMIT_Z_DOWN_GPIO_Port,
		.back_down_limit_pin = LIMIT_Z_DOWN_Pin,
		.back_down_limit_active_level = 0,
		.front_up_limit_gpio_port = LIMIT_Z_UP_GPIO_Port,
		.front_up_limit_pin = LIMIT_Z_UP_Pin,
		.front_up_limit_active_level = 0,
		.enable_gpio_port = Z_ENABLE_GPIO_Port,
		.enable_pin = Z_ENABLE_Pin,
		.enable_active_level = 0,
		.enable_state = 0,
		.dir_gpio_port = Z_DIR_GPIO_Port,
		.dir_pin = Z_DIR_Pin,
		.step_gpio_port = Z_STEP_GPIO_Port,
		.step_pin = Z_STEP_Pin,
		.forward_dir = CW,
		.timer = &htim9,
		.uart = &huart2,
		.tmc_addr = 0,
		.rampHome.speed = PARAMETER_HOME_SPEED_Z,
		.rampHome.accel = PARAMETER_HOME_ACCEL_Z,
		.rampHome.decel = PARAMETER_HOME_DECEL_Z,
		.rampPrint.speed = PARAMETER_PRINT_SPEED_Z,
		.rampPrint.accel = PARAMETER_PRINT_ACCEL_Z,
		.rampPrint.decel = PARAMETER_PRINT_DECEL_Z,
		.rampJog.speed = PARAMETER_JOG_SPEED_Z,
		.rampJog.accel = PARAMETER_JOG_ACCEL_Z,
		.rampJog.decel = PARAMETER_JOG_DECEL_Z,
		.rampMove.speed = PARAMETER_MOVE_SPEED_Z,
		.rampMove.accel = PARAMETER_MOVE_ACCEL_Z,
		.rampMove.decel = PARAMETER_MOVE_DECEL_Z,
		.home_timeout = 20000,
		.standby_position = 10000,
		.not_safe = 0,
		.safe_dir = 0
};

tPrinter Printer = {
		.emergency_gpio_port = EMERGENCY_GPIO_Port,
		.emergency_pin = EMERGENCY_Pin,
		.emergency_pin_active_level = 1,
		.pe_signal_gpio_port = OPE_SIGNAL_GPIO_Port,
		.pe_signal_pin = OPE_SIGNAL_Pin,
		.pe_signal_active_level = 1,
		.photo_barier_gpio_port = PHOTO_SENSOR_GPIO_Port,
		.photo_barier_pin = PHOTO_SENSOR_Pin,
		.photo_barier_active_level = 1,
		.initilize_state = 0,
		.emergency_state = 0,
		.MotionY = &MotionY,
		.MotionZ = &MotionZ,
		.timeout_motion_z = PARAMETER_PRINT_TIMEOUT_Z,
		.timeout_motion_y = PARAMETER_PRINT_TIMEOUT_Y,
		.service_mode = 0,
		.photo_barier_state = 1,
		.printer_state = IDLE,
		.pe_lower_limit = PARAMETER_PRINT_PE_LOWER_LIMIT, //For Epson P600, todo need to be adjusted
		.pe_upper_limit = PARAMETER_PRINT_PE_UPPER_LIMIT, //For Epson P600, todo need to be adjusted
		.print_start_position = PARAMETER_PRINT_START_POS, //todo need to be adjusted
		.stepper_factor = PARAMETER_PRINT_STEPPER_FACTOR, //todo need to be adjusted
		.enc_timer = TIM2,
		.eject_trigger_position = PARAMETER_PRINT_EJECT_TRIGGER_POS, //For Epson P600, todo need to be adjusted
		.eject_position = PARAMETER_PRINT_EJECT_POS //todo need to be adjusted for CJ200 printer
};

//Functions Prototype
void transmitShim(tCMDInt*, uint8_t*, uint8_t);

static void taskHandleUSBCommunication()
{
	uint32_t waitingBytes;
	uint8_t c = 0;
	/*
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
				uint8_t c = serialReadByte(serialPort);
				if (c == '#') {
					cliEnter(serialPort);
					//disable minMode
					minMode = 0;
				}

				min_poll(&min_pc, (uint8_t *)c, 1);
			}
		}
    }
    */
	waitingBytes = serialRxBytesWaiting(serialPort);

	//
	if(waitingBytes == 0)
	{
			min_poll(&min_pc, &c, 0);
	}

	//Tutaj jest powa�ny problem poniewa� zawiesi aplikacj� w momencie gdy du�a ilosc danych bedzie wysylala
	//Moze sie okazac ze usb nie bedzie w stanie wysla odpiwednio duzo infromacji

	//Po czasie kiedy wszystkie wiadomosci wejsciowe zostana obsluzone, oprogramowanie sie odwiesza nie wystepuja problemy z wywaleniem aplikacji
	while(waitingBytes)
	{
		uint8_t c = serialReadByte(serialPort);
		min_poll(&min_pc, &c, 1);
		waitingBytes--;
	}

}

//Handle communication with UART
static void taskHandleUART8()
{
	int c = 0;

	do {

	c = UARTDMA_GetCharFromBuffer(&huartdma);

	if(c == -1)
	{
			min_poll(&min_hmi, (uint8_t *)&c, 0);
			break;
	}

	min_poll(&min_hmi, (uint8_t *)&c, 1);

	}while(true);

	//Check if some data to send
	UARTDMA_TxBufferFlush(&huartdma);
}

void taskMotionProcess()
{
	MotionProcess(&MotionZ);
	MotionProcess(&MotionY);
}

void taskPrinterProcess()
{
	PrinterProcess(&Printer);
	PrinterPrintingProcess(&Printer);
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
}

void tasksInitialize()
{
	SchedulerInit(&stsTasks);

	min_init_context(&min_pc, 0);
	min_init_context(&min_hmi, 1);

	commandInit(&CMD, (transmitFunc_t)transmitShim);
	M24C0XInitialize(&eeprom);
	serialPort = usbVcpOpen();

	TaskCreate(&stsTasks, &tLedTask, &led, 255);
	TaskStartRepeatedly(&tLedTask, 500);

	//TaskCreate(&stsTasks, &tCore, &Core, 5);
	//TaskStartRepeatedly(&tCore, 2000);

	TaskCreate(&stsTasks, &tHandleUSBCommunication, &taskHandleUSBCommunication, 50);
	TaskStartRepeatedly(&tHandleUSBCommunication, 10);

	TaskCreate(&stsTasks, &tHandleUART8, &taskHandleUART8, 49);
	TaskStartRepeatedly(&tHandleUART8, 10);

	TaskCreate(&stsTasks, &tMotionProcess, &taskMotionProcess, 10);
	TaskStartRepeatedly(&tMotionProcess, 50);

	TaskCreate(&stsTasks, &tPrinterProcess, &taskPrinterProcess, 5);
	TaskStartRepeatedly(&tPrinterProcess, 5);

	deactivePE(&Printer);

	MotionControllerInitialize(&MotionZ);
	MotionControllerInitialize(&MotionY);
	MotionSetIRUN(&MotionY,25);
	MotionSetIHOLD(&MotionY,15);
}

void tasksScheduler()
{
	Scheduler(&stsTasks);
}

////////////////////////////////// CALLBACKS ///////////////////////////////////

uint16_t min_tx_space(uint8_t port)
{
	uint16_t n = 0;
	if(port == 0)
	{
		n = serialTxBytesFree(serialPort);
	}
	else if(port == 1)
	{
		n = UARTDMA_TxBufferSpace(&huartdma);
	}

	return n;
}

// Send a character on the designated port.
void min_tx_byte(uint8_t port, uint8_t byte)
{
	if(port == 0)
	{
			serialWriteBuf(serialPort, &byte, 1);
	} else if(port == 1)
	{
			UARTDMA_TxBufferAppend(&huartdma, byte);
	}
}

// Tell MIN the current time in milliseconds.
uint32_t min_time_ms(void)
{
  return Ticks();
}

void transmitShim(tCMDInt *cmd_i, uint8_t *data, uint8_t len)
{
	  bool result = min_queue_frame(&min_hmi, cmd_i->min_id, data, len);
	  if(!result) {
	    //Serial.println("Queue failed");
	  }
}

void min_application_handler(uint8_t min_id, uint8_t *min_payload, uint8_t len_payload, uint8_t port)
{
	//Send payload to command interpreter
	CMD.len_payload = len_payload;
	CMD.min_id = min_id;
	CMD.min_payload = min_payload;
	CMD.port = port;

	commandInterpreter(&CMD);

	//Just for SOAK test
	/*
  // In this simple example application we just echo the frame back when we get one
  bool result = min_queue_frame(&min_hmi, min_id, min_payload, len_payload);
  if(!result) {
    //Serial.println("Queue failed");
  }
  */
}

void min_tx_start(uint8_t port)
{
	return;
}
void min_tx_finished(uint8_t port)
{
	return;
}

// UART DMA Callback
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	UARTDMA_DmaTransmitionCompletedIrq(&huartdma);
}
