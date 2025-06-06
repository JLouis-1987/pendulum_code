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

#define SR_TXE			(1U<<7) 	//Transmit data register is empty not mask
#define SR_RXNE			(1U<<5) 	//Read data register is empty not mask

void uart2_tx_init (void);
void uart2_rxtx_interrupt_init(void);
void uart2_rxtx_init (void);
uint32_t rx_buffer_count(void);
uint8_t rx_buffer_read(void);
void uart_rx_callback(void);
void uart_tx_callback(void);
char uart2_read(void);

#define SR_RXNE			(1U<<5) 	//Read data register is empty not mask
#endif /* UART_H_ */
