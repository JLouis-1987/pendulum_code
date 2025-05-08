#include "led.h"

#define GPIODEN				(1U<<3) //Enable clock for GPIO D

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


void init_leds(void)
{
	RCC->AHB1ENR |= GPIODEN;

	//Set PA12 as alternate mode Pin
	GPIOD->MODER &=~(1U<<24);
	GPIOD->MODER |= (1U<<25);
	//Set PA12 alternate mode to AF2 (Timer 2 CH 1)
	GPIOD->AFR[1] |= (0b0010<<16);
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
