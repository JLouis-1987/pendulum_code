#include "led.h"

#define GPIOCEN				(1U<<2) //Enable clock for GPIO C
#define GPIODEN				(1U<<3) //Enable clock for GPIO D


#define PIN6				(1U<<6)
#define PIN12 				(1U<<12)
#define PIN13 				(1U<<13)
#define PIN14 				(1U<<14)
#define PIN15 				(1U<<15)

#define LED_GREEN_ON		PIN12
#define LED_GREEN_OFF		(PIN12<<16)
#define LED_ORANGE_ON		PIN13
#define LED_ORANGE_OFF		(PIN13<<16)
#define LED_RED_ON			PIN14
#define LED_RED_OFF			(PIN14<<16)
#define LED_BLUE_ON			PIN15
#define LED_BLUE_OFF		(PIN15<<16)

#define ALTFUN_AF2 			(0b0010)
#define ALTFUN_AF3			(0b0011)
#define ALTFUN_6_14_MASK	0x00f000000
#define ALTFUN_4_12_MASK	0x0000f0000

void init_PC6(void)
{
	RCC->AHB1ENR |= GPIOCEN;

	//Set PC6 as alternate mode Pin
	GPIOC->MODER &=~(1U<<12);
	GPIOC->MODER |= (1U<<13);
	//Set PC6 alternate mode to AF3 (Timer 8 CH 1)
	GPIOC->AFR[0]  = (GPIOC->AFR[0]  & ~ALTFUN_6_14_MASK) | (ALTFUN_AF3<<24);

}


void init_leds(void)
{
	RCC->AHB1ENR |= GPIODEN;

	//Set PA12 as alternate mode Pin
	GPIOD->MODER &=~(1U<<24);
	GPIOD->MODER |= (1U<<25);
	//Set PA12 alternate mode to AF2 (Timer 2 CH 1)
	GPIOD->AFR[1] |= (GPIOC->AFR[1]  & ~ALTFUN_4_12_MASK) | (ALTFUN_AF2<<16);
	//Set PA13 as output Pin
	GPIOD->MODER |= (1U<<26);
	GPIOD->MODER &=~(1U<<27);
	//Set PA14 as output Pin
	GPIOD->MODER |= (1U<<28);
	GPIOD->MODER &=~(1U<<29);
	//Set PA15 as output Pin
	GPIOD->MODER |= (1U<<30);
	GPIOD->MODER &=~(1U<<31);
}

void cycle_leds(void)
{
	GPIOD->BSRR  = LED_GREEN_ON;
	for(long i=0; i<100000; i++) {}
	GPIOD->BSRR  = LED_ORANGE_ON;
	for(long i=0; i<100000; i++) {}
	GPIOD->BSRR  = LED_RED_ON;
	for(long i=0; i<100000; i++) {}
	GPIOD->BSRR  = LED_BLUE_ON;
	for(long i=0; i<400000; i++) {}
	GPIOD->BSRR  = LED_BLUE_OFF;
	for(long i=0; i<100000; i++) {}
	GPIOD->BSRR  = LED_RED_OFF;
	for(long i=0; i<100000; i++) {}
	GPIOD->BSRR  = LED_ORANGE_OFF;
	for(long i=0; i<100000; i++) {}
	GPIOD->BSRR  = LED_GREEN_OFF;
	for(long i=0; i<400000; i++) {}

}
