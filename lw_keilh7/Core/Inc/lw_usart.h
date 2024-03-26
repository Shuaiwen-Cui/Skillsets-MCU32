#ifndef __LW_USART_H
#define __LW_USART_H

	#include "main.h"

	#define USART1_BaudRate               115200
	
	#define	USART1_TX_PORT								GPIOA//TX
	#define	USART1_RX_PORT								GPIOA// RX
	#define USART1_RX_PIN									GPIO_PIN_10//RX 
	#define USART1_TX_PIN									GPIO_PIN_9// TX

	#define GPIO_USART1_TX_CLK_ENABLE     __HAL_RCC_GPIOA_CLK_ENABLE()// TX pin clock
	#define GPIO_USART1_RX_CLK_ENABLE     __HAL_RCC_GPIOA_CLK_ENABLE()// RX pin clock

	void lw_usart1_init(void);	// USART1 initialization function

#endif





