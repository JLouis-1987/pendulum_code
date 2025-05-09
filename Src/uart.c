/*
 * uart.c
 *
 *  Created on: May 5, 2025
 *      Author: gaspr
 */

#include "uart.h"
#include "clock.h"
#include "utils/ringbuffer.h"

#define GPIOAEN			(1U<<0) 	//Enable clock for GPIO A on AHB1
#define UART2EN			(1U<<17) 	//Enable clock for UART2 on APB1
#define CR1_TE			(1U<<3) 	//Enable Transmit for UART
#define CR1_RE			(1U<<2) 	//Enable Receive for UART
#define CR1_UE			(1U<<13)	//Enable UART Module.

#define RXNEIE			(1U<<5)		//Enable RX interrupt
#define TXEIE			(1U<<7)		//Enable TX interrupt

#define UART_BAUDRATE 	115200
#define RING_BUFFER_SIZE	64

//static ring_buffer_t rb_tx = {0U};
//static ring_buffer_t rb_rx = {0U};
//static uint8_t data_buffer[RING_BUFFER_SIZE] = {0U};

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk,
		uint32_t BaudRate);
static uint16_t compute_uart_DIV(uint32_t PeriphClk, uint32_t BaudRate);
void uart2_write(int ch);

//static void uart_tx_clear_interrupt();


int __io_putchar(int ch)
{
	//put data into the ring buffer
	//ringbuf_push(ch, tx_buffer);
	//enable the TX interrupt
	//USART2->CR2 |= TXEIE;
	//return ch;

	uart2_write(ch);
	return ch;

}
/*
static void uart_rx_callback(void)
{

	ringbuf_push(USART2->DR, rxbuffer);
}

static void uart_tx_callback(void)
{
	if (ringbuf_is_empty(*txbuffer)){
		while(1);//need to add handling for this
	}


}
*/

//not used
char uart2_read(void) {
	//make sure receive data register is empty
	while (!(USART2->SR & SR_RXNE)) {
	}
	//write to transmit data register
	return USART2->DR;
}

void uart2_write(int ch) {
	//make sure transmit data register is empty

	while (!(USART2->SR & SR_TXE)) {
	}
	//write to transmit data register
	USART2->DR = (ch & 0xFF);
	for (int i = 0; i < 100; i++) {}
}

void uart2_rxtx_interrupt_init(void) {

	//Init ring buffer
	//ringbuf_init(*tx_buffer);
	//ringbuf_init(*rx_buffer);

	//Configure the UART GPIO pins
	//enable clock access
	RCC->AHB1ENR |= GPIOAEN;
	//set mode of PA2 Pin to alternate function (0x10).
	GPIOA->MODER &= ~(1U << 4);
	GPIOA->MODER |= (1U << 5);
	//set PA2 alternate function type to UART_TX (AF07 = 0X0111) from alternate function table
	GPIOA->AFR[0] |= (1u << 8);
	GPIOA->AFR[0] |= (1u << 9);
	GPIOA->AFR[0] |= (1u << 10);
	GPIOA->AFR[0] &= ~(1U << 11);

	//set mode of PA3 Pin to alternate function (0x10).
	GPIOA->MODER &= ~(1U << 6);
	GPIOA->MODER |= (1U << 7);
	//set PA3 alternate function type to UART_RX (AF07 = 0X0111) from alternate function table
	GPIOA->AFR[0] |= (1u << 12);
	GPIOA->AFR[0] |= (1u << 13);
	GPIOA->AFR[0] |= (1u << 14);
	GPIOA->AFR[0] &= ~(1U << 15);

	//Configure UART module
	RCC->APB1ENR |= UART2EN; 				//enable clock access to UART2
	uart_set_baudrate(USART2, APB1FREQ, UART_BAUDRATE); //configure the UART Baud rate
	USART2->CR1 = (CR1_TE | CR1_RE); //Sets Transmit enable and cleans everything else (8 bit, even parity)

	/*Enable RX TX interrupts*/
	USART2->CR1 |= RXNEIE;
	//clear TX buffer and flag by reading the buffer
	USART2->DR;

	/*enable UART2 interrupt in NVIC*/
	NVIC_EnableIRQ(USART2_IRQn);

	USART2->CR1 |= CR1_UE; 				//enable the UART module
}

static void uart_set_baudrate(USART_TypeDef *USARTx, uint32_t PeriphClk,
		uint32_t BaudRate) {
	int UART_DIV = compute_uart_DIV(PeriphClk, BaudRate);
	USARTx->BRR = UART_DIV;
}

static uint16_t compute_uart_DIV(uint32_t PeriphClk, uint32_t BaudRate) {
	return PeriphClk / BaudRate;

}
