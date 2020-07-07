#include "UART_DMA.h"

void UARTDMA_UartIrqHandler(UARTDMA_HandleTypeDef *huartdma)
{
	if(huartdma->huart->Instance->SR & UART_FLAG_IDLE)       // Check if Idle flag is set
	{
		volatile uint32_t tmp;
		tmp = huartdma->huart->Instance->SR;                      // Read status register
		tmp = huartdma->huart->Instance->DR;                      // Read data register
		huartdma->huart->hdmarx->Instance->CR &= ~DMA_SxCR_EN; // Disable DMA - it will force Transfer Complete interrupt if it's enabled
		UNUSED(tmp);
	}
}

void UARTDMA_DmaRxIrqHandler(UARTDMA_HandleTypeDef *huartdma)
{
	uint8_t *UartBufferPointer, *DmaBufferPointer;
	uint32_t Length;
	uint16_t i, TempHead;

	typedef struct
	{
		__IO uint32_t ISR;   // DMA interrupt status register
		__IO uint32_t Reserved0;
		__IO uint32_t IFCR;  // DMA interrupt flag clear register
	} DMA_Base_Registers;

	DMA_Base_Registers *DmaRegisters = (DMA_Base_Registers *) huartdma->huart->hdmarx->StreamBaseAddress; // Take registers base address

	if (__HAL_DMA_GET_IT_SOURCE(huartdma->huart->hdmarx, DMA_IT_TC) != RESET) // Check if interrupt source is Transfer Complete
	{
		DmaRegisters->IFCR = DMA_FLAG_TCIF0_4 << huartdma->huart->hdmarx->StreamIndex;	// Clear Transfer Complete flag

		Length = DMA_RX_BUFFER_SIZE - huartdma->huart->hdmarx->Instance->NDTR; // Get the Length of transfered data

		UartBufferPointer = huartdma->UART_Buffer;
		DmaBufferPointer = 	huartdma->DMA_RX_Buffer;

		// Write received data for UART main buffer - circular buffer
		for(i = 0; i < Length; i++)
		{
			TempHead = (huartdma->UartRxBufferHead + 1) % UART_RX_BUFFER_SIZE;
			if(TempHead == huartdma->UartRxBufferTail)
			{
				huartdma->UartRxBufferHead = huartdma->UartRxBufferTail;	// No room for new data
			}
			else
			{
				UartBufferPointer[TempHead] = DmaBufferPointer[i];
				huartdma->UartRxBufferHead = TempHead;
			}
		}

		DmaRegisters->IFCR = 0x3FU << huartdma->huart->hdmarx->StreamIndex; 		// Clear all interrupts
		huartdma->huart->hdmarx->Instance->M0AR = (uint32_t) huartdma->DMA_RX_Buffer; // Set memory address for DMA again
		huartdma->huart->hdmarx->Instance->NDTR = DMA_RX_BUFFER_SIZE; // Set number of bytes to receive
		huartdma->huart->hdmarx->Instance->CR |= DMA_SxCR_EN;            	// Start DMA transfer
	}
}

int UARTDMA_GetCharFromBuffer(UARTDMA_HandleTypeDef *huartdma)
{
	if(huartdma->UartRxBufferHead == huartdma->UartRxBufferTail)
	{
		return -1; // error - no char to return
	}
	huartdma->UartRxBufferTail = (huartdma->UartRxBufferTail + 1) % UART_RX_BUFFER_SIZE;

	return huartdma->UART_Buffer[huartdma->UartRxBufferTail];
}

void UARTDMA_Init(UARTDMA_HandleTypeDef *huartdma, UART_HandleTypeDef *huart)
{
	huartdma->huart = huart;
	huartdma->UartDuringTransmition = 0;
	huartdma->UartRxBufferHead = 0;
	huartdma->UartRxBufferTail = 0;
	huartdma->UartTxBufferHead = 0;

	__HAL_UART_ENABLE_IT(huartdma->huart, UART_IT_IDLE);   	// UART Idle Line interrupt
	__HAL_DMA_ENABLE_IT(huartdma->huart->hdmarx, DMA_IT_TC); // UART DMA Transfer Complete interrupt

	HAL_UART_Receive_DMA(huartdma->huart, huartdma->DMA_RX_Buffer, DMA_RX_BUFFER_SIZE); // Run DMA for whole DMA buffer

	huartdma->huart->hdmarx->Instance->CR &= ~DMA_SxCR_HTIE; // Disable DMA Half Complete interrupt
}

uint16_t UARTDMA_TxBufferSpace(UARTDMA_HandleTypeDef *huartdma)
{
	if(huartdma->UartDuringTransmition)
		{
			return 0;
		}

	return (UART_TX_BUFFER_SIZE - 1) - huartdma->UartTxBufferHead;
}
void UARTDMA_TxBufferAppend(UARTDMA_HandleTypeDef *huartdma, uint8_t byte)
{
	if(!huartdma->UartDuringTransmition)
	{
		huartdma->UART_TX_Buffer[huartdma->UartTxBufferHead++] = byte;

		if(huartdma->UartTxBufferHead >= UART_TX_BUFFER_SIZE)
		{
				UARTDMA_TxBufferFlush(huartdma);
		}
	}
}
void UARTDMA_TxBufferFlush(UARTDMA_HandleTypeDef *huartdma)
{
	HAL_StatusTypeDef status;
	if(!huartdma->UartDuringTransmition && huartdma->UartTxBufferHead!= 0)
	{
		status = HAL_UART_Transmit_DMA(huartdma->huart, huartdma->UART_TX_Buffer, huartdma->UartTxBufferHead);
		if(status == HAL_OK)
		{
				huartdma->UartDuringTransmition = 1;
		} else {
				//Can't send data
				//TODO Error handling
		}
	}
}
void UARTDMA_DmaTransmitionCompletedIrq(UARTDMA_HandleTypeDef *huartdma)
{
	if(huartdma->UartDuringTransmition)
		{
			huartdma->UartDuringTransmition = 0;
			huartdma->UartTxBufferHead = 0;
		}
}
