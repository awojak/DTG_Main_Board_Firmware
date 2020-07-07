
#include "stm32f4xx_hal.h"

#define DMA_RX_BUFFER_SIZE          64
#define UART_RX_BUFFER_SIZE            2048
#define UART_TX_BUFFER_SIZE 				2048

typedef struct
{
	UART_HandleTypeDef* huart;					// UART handler

	uint8_t DMA_RX_Buffer[DMA_RX_BUFFER_SIZE];	// DMA direct buffer
	uint8_t UART_Buffer[UART_RX_BUFFER_SIZE];		// UART working circular buffer

	uint8_t UART_TX_Buffer[UART_TX_BUFFER_SIZE];

	uint16_t UartRxBufferHead;
	uint16_t UartRxBufferTail;

	uint16_t UartTxBufferHead;
	uint8_t UartDuringTransmition;

}UARTDMA_HandleTypeDef;

void UARTDMA_UartIrqHandler(UARTDMA_HandleTypeDef *huartdma);
void UARTDMA_DmaRxIrqHandler(UARTDMA_HandleTypeDef *huartdma);
void UARTDMA_Init(UARTDMA_HandleTypeDef *huartdma, UART_HandleTypeDef *huart);

uint16_t UARTDMA_TxBufferSpace(UARTDMA_HandleTypeDef *huartdma);
void UARTDMA_TxBufferAppend(UARTDMA_HandleTypeDef *huartdma, uint8_t byte);
void UARTDMA_TxBufferFlush(UARTDMA_HandleTypeDef *huartdma);
void UARTDMA_DmaTransmitionCompletedIrq(UARTDMA_HandleTypeDef *huartdma);
int UARTDMA_GetCharFromBuffer(UARTDMA_HandleTypeDef *huartdma);
