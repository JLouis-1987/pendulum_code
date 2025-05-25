/*
 * i2c.c
 *
 *  Created on: May 24, 2025
 *      Author: gaspr
 */

#include <stdint.h>
#include "stm32f4xx.h"

#define MODER_7_MASK 		(0x0000C000)
#define MODER_8_MASK 		(0x00030000)
#define MODER_ALTFUN		(0b10)
#define ALTFUN_7_15_MASK 	(0xF0000000)
#define ALTFUN_0_8_MASK 	(0x0000000F)
#define ALTFUN_AF4		 	(0b0100)
#define PUPDR_7_MASK 		(0b01000000)
#define PUPDR_8_MASK 		(0b10000000)
#define PUPDR_UP	 		(0b01)

#define I2CFREQMASK 		(0x1F)
#define I2CFREQ 			(36) 	//APB1 Clock is at 36 MHz
#define I2CCCR				(180)	//5000 ns / 27.778 ns (T high/ P clock)
#define I2CCCRMASK			(0xFFF)
#define I2CTRISE			(36)	//1000 ns / 27.778 ns (T high/ P clock)
#define I2CTRISEMASK		(0x1F)

/*
 * I2C1
 * PB8 --> SCL
 * PB7 --> SDA
 *
 */

static void I2C1_Start(char paddr, char maddr);

void I2C1_Init(void)
{
	/*
	 * Function: I2C1_Init
	 * ----------------------
	 * This function initialize the I2C1 peripheral using Pins PB7(SDA) and PB8 (SCL).
	 * Assumes the APB1 clock frequency is 36 MHz
	 *
	 * Parameters:
	 *   None
	 *
	 * Returns:
	 *   None
	 */

	/*Enable APB1 clock Access*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	/*Enable clock access to I2C1 */
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

	/*** Pin Setup **/
	/*Set PB8 and PB7 as alternate mode Pin*/
	GPIOB->MODER  = (GPIOB->MODER  & ~MODER_7_MASK) | (MODER_ALTFUN<<14);
	GPIOB->MODER  = (GPIOB->MODER  & ~MODER_8_MASK) | (MODER_ALTFUN<<16);

	/*Set Port to Open Drain with pull up*/
	GPIOB->OTYPER |= (1U<<7);
	GPIOB->OTYPER |= (1U<<8);

	GPIOB->PUPDR  = (GPIOB->PUPDR  & ~PUPDR_7_MASK) | (PUPDR_UP<<7);
	GPIOB->PUPDR  = (GPIOB->PUPDR  & ~PUPDR_8_MASK) | (PUPDR_UP<<9);

	/*Set PB8 and PB7 to alternate mode to AF4 (I2C1 SCL and SDA)*/
	GPIOB->AFR[1]  = (GPIOB->AFR[1]  & ~ALTFUN_0_8_MASK) | (ALTFUN_AF4<<0);
	GPIOB->AFR[0]  = (GPIOB->AFR[0]  & ~ALTFUN_7_15_MASK) | (ALTFUN_AF4<<28);

	/**** Set up i2c1 ***/
	/*reset i2c before setting it up */
	I2C1->CR1 |= I2C_CR1_SWRST;
	I2C1->CR1 &= ~I2C_CR1_SWRST;

	/*disable i2c before configuration*/
	I2C1->CR1 &= ~I2C_CR1_PE;

	/*set i2c1 timing parameters */
	I2C1->CR2 = (I2C1->CR2  & ~I2CFREQMASK) | (I2CFREQ); 		//Set the frequency
	I2C1->CCR = (I2C1->CCR  & ~I2CCCRMASK) | (I2CCCR);   		//Set the CCR so I2C runs at 100kHZ
	I2C1->TRISE = (I2C1->TRISE  & ~I2CTRISEMASK) | (I2CTRISE); 	//Set the rise to time to 1000 ns

	I2C1->CR1 |= I2C_CR1_PE;   //enable i2c after configuration
}

void I2C1_byteRead(char paddr, char maddr, char* data)
{
	/*
	 * Function: I2C1_byteRead
	 * ----------------------
	 * This function read a single byte from the the i2c device at the device and
	 * memory address specified. Writes the received data to the provide pointer.
	 *
	 * Parameters:
	 *   char paddr - 7bit device address
	 *   char maddr - 8 bit memory address
	 *   char data - pointer to a char that will receive the data.
	 *
	 * Returns:
	 *   None
	 */

	I2C1_Start(paddr, maddr);

	while(!(I2C1->SR1 & I2C_SR1_TXE)){} //wait for i2c Transmit bit to be empty

	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){} //wait for i2c start bit to be set

	/*transmit Slave address + read*/
	I2C1->DR = paddr<<1|1;
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){} //wait for i2c addr bit to be set

	/*disable the acknowledge*/
	I2C1->CR1 &= ~I2C_CR1_ACK;

	(void)I2C1->SR2; 				 //read SR2 to clear ADDR flag

	/* Generate Stop after Data is received*/
	 I2C1->CR1 |= I2C_CR1_STOP;

	while(!(I2C1->SR1 & I2C_SR1_RXNE)){} //wait for i2c RXNE bit to be set

	*data = I2C1->DR;

}

void I2C1_Read(char paddr, char maddr, int16_t n, char* data)
{
	/* Function: I2C1_Read
	 * ----------------------
	 * This function read multiple bytes from the the i2c device starting at the maddr
	 * specified. Writes the received data to the provide pointer.
	 *
	 * Parameters:
	 *   char paddr - 7bit device address
	 *   char maddr - 8 bit memory address
	 *   int16_t n  - number of bytes to read
	 *   char data - pointer to a char that will receive the data.
	 *
	 * Returns:
	 *   None
	 */

	I2C1_Start(paddr, maddr);

	while(!(I2C1->SR1 & I2C_SR1_TXE)){} //wait for i2c Transmit bit to be empty

	I2C1->CR1 |= I2C_CR1_START;  //Generate restart
	while(!(I2C1->SR1 & I2C_SR1_SB)){} //wait for i2c start bit to be set

	/*transmit Slave address + read*/
	I2C1->DR = paddr<<1|1;
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){} //wait for i2c addr bit to be set

	(void)I2C1->SR2; 				 //read SR2 to clear ADDR flag

	/*Enable the acknowledge*/
	I2C1->CR1 |= I2C_CR1_ACK;

	while(n > 0U)
	{
		/*if one byte*/
		if(n == 1U){
			/*disable the acknowledge*/
			I2C1->CR1 &= ~I2C_CR1_ACK;

			/*Generate Stop after Data is received*/
			I2C1->CR1 |= I2C_CR1_STOP;

			/*wait for i2c RXNE bit to be set*/
			while(!(I2C1->SR1 & I2C_SR1_RXNE)){}
			*data++ = I2C1->DR;
			break;
		}
		else{
			/*wait for i2c RXNE bit to be set*/
			while(!(I2C1->SR1 & I2C_SR1_RXNE)){}
			*data++ = I2C1->DR;
			n--;
		}
	}
}


void I2C1_Write(char paddr, char maddr, int n, char* data)
{
	/* Function: I2C1_Write
	 * ----------------------
	 * This function writes multiple bytes to the the i2c device starting at the maddr
	 * specified. Writes the received data to the provide pointer.
	 *
	 * Parameters:
	 *   char paddr - 7bit device address
	 *   char maddr - 8 bit memory address
	 *   int16_t n  - number of bytes to write
	 *   char data - pointer to a char that will receive the data.
	 *
	 * Returns:
	 *   None
	 */
	I2C1_Start(paddr, maddr);

	for(int i = 0; i < n; i++){
		while(!(I2C1->SR1 & I2C_SR1_TXE)){} //wait for i2c Transmit bit to be empty
		I2C1->DR = *data++;
		while(!(I2C1->SR1 & I2C_SR1_BTF)){} //wait for i2c BTF bit to be set
	}

	/* Generate Stop after Data is received*/
	 I2C1->CR1 |= I2C_CR1_STOP;
}

void I2C1_Start(char paddr, char maddr)
{
	/*
	 * Function: I2C1_Start
	 * ----------------------
	 * This function starts the i2c communication to the device. The function does
	 * the following:
	 * -waits for bus to not be busy
	 * -asserts the start
	 * -sends the device address
	 * -sends the memory address
	 *
	 * Parameters:
	 *   char paddr - 7bit device address
	 *   char maddr - 8 bit memory address
	 *
	 * Returns:
	 *   None
	 */

	/*wait for i2c to stop being busy*/
	while(I2C1->SR2 & I2C_SR2_BUSY){}

	/*set and wait for i2c start bit to be set*/
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB)){}

	/*transmit Slave address, in write mode*/
	I2C1->DR = paddr<<1;

	/*wait for and then clear Address bit*/
	while(!(I2C1->SR1 & I2C_SR1_ADDR)){}
	(void)I2C1->SR2;

	//wait for i2c Transmit bit to be empty
	while(!(I2C1->SR1 & I2C_SR1_TXE)){}
	I2C1->DR = maddr;
}

