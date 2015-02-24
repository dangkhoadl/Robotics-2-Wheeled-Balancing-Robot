/* ==================================================================================
Description: 	Declare defines and functions for UART1 for sending data to PC.
Target: 	    STM32F407
------------------------------------------------------------------------------------*/
#ifndef __UART_DMA_H__
#define __UART_DMA_H__

#include "stm32f4xx.h"


/* Exported functions ------------------------------------------------------- */
void UART_DMA_CONFIG(uint8_t *TxAddr,uint8_t Tx_size,uint8_t *RxAddr, uint8_t Rx_size, uint32_t baudrate);
void USART_config (void);
void DMA_TX(void);
void SENDDATA(float Value, uint8_t* TxAddr, uint8_t channel);

#endif  // __UART_DMA_H__
