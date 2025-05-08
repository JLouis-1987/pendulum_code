
#include <stdint.h>
#include "stm32f4xx.h"

#define HSEON			(1U<<16)
#define HSERDY			(1U<<17)
#define PWREN			(1U<<28)
#define VOS				(1U<<14)
#define LATENCY_2 		0b010
#define LATENCY_MASK	0b111
#define PRFTEN			(1U<<8)
#define PPRE1			(0b100<<13)	//set APB1 clock to be divided by 2 (36 MHz)
#define PPRE2 			(0b000<<10)	//set APB2 clock to be divided by 0 (72 MHz)
#define HPRE			(0b000<<4)	//set AHB clock to be divided by 0 (72 MHz)
#define PRESCALER		PPRE1 | PPRE2 | HPRE
#define PRESCALER_MASK	0xFCF0
#define PLLSRC 			(1U<<22)	//set PLL source to HSE
#define PLLP			(0b01<<16) 	//set P divisor to 4 (assumes 288MHz VCO output freq)
#define PLLP_MASK		(0b11<<16)
#define PLLN 			(144 <<6)	//set N divisor (Sets VCO OUTPUT frequency to 288 Mhz)
#define PLLM 			4			//set M divisor (Sets VCO input frequency to 2 Mhz)
#define PLLN_M 			PLLN | PLLM
#define PLLN_M_MASK 	(0x7FFF)
#define PLLON 			(1U<<24)
#define PLLRDY 			(1U<<25)
#define SW 				0b10		//set clock source to PLL
#define SW_MASK			0b11
#define SWS 			(0b10<<2)	//clock source status bits
#define SWS_MASK		(0b11<<2)

uint16_t PllCFG = 0;

void clock_init(void)
{
	/*function sets clock to use the 8 MHz external oscillator, and the
	 * system clock to 72 MHz.
     */

	//Set flash wait states and enable pre-fetch
	FLASH->ACR = (FLASH->ACR & ~LATENCY_MASK) | LATENCY_2; //Latency set to 2 wait states per data sheet
	FLASH->ACR |= PRFTEN;

	RCC->CFGR =  (RCC->CFGR & ~PRESCALER_MASK) | PRESCALER;

	// ENABLE HSE and wait for the HSE to become Ready
	RCC->CR |= HSEON;
	while(!(RCC->CR & HSERDY));

	// Set the POWER ENABLE CLOCK and VOLTAGE REGULATOR
	RCC->APB1ENR |= PWREN;
	PWR->CR |= VOS;

	RCC->PLLCFGR |= PLLSRC; //Set source of PLL

	RCC->PLLCFGR =  (RCC->PLLCFGR & ~PLLN_M_MASK) | PLLN_M; // Set PLL M and N multipliers
	RCC->PLLCFGR =  (RCC->PLLCFGR & ~PLLP) | PLLP; // Set PLL P multipliers
	RCC->CR |= PLLON; //turn on PLL
	while(!(RCC->CR & PLLRDY)); //wait for PLL Ready flag

	RCC->CFGR = (RCC->CFGR & ~SW_MASK) | SW;	//set clock source

	while(!((RCC->CFGR & SWS_MASK) == SWS)); //wait for PLL to be clock source

}
