/*
 * uart.h
 *
 *  Created on: Apr 23, 2025
 *      Author: gaspr
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>
#include "stm32f4xx.h"

void uart2_tx_init (void);
void uart2_rxtx_interrupt_init(void);
void uart2_rxtx_init (void);
char uart2_read(void);

#define SR_RXNE			(1U<<5) 	//Read data register is empty not mask
#endif /* UART_H_ */
