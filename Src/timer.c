#include "stm32f4xx.h"

#define TIM4EN 	(1U<<2)
#define TIM8EN 	(1U<<1)
#define CR1_CEN (1U<<0)
#define OC1M_TOGGLE (0b011<<4)
#define CC1E_ENABLE (1U<<0)

void tim8_output_compare(void)
{
	/*Enable Clock access to timer 8*/
	RCC->APB2ENR |= TIM8EN;
	/* set prescaler value*/
	TIM8->PSC = 72 - 1;   //72 000 000/72 = 1 000 000
	/*Set Auto reload value*/
	TIM8->ARR = 1000 - 1;  //1 000 000/1 000 = 0.001 Hz

	/*Set Output compare toggle mode*/
	TIM8->CCMR1 |= OC1M_TOGGLE;
	/*enable timer 8 Ch 1 in compare mode*/
	TIM8->CCER |= CC1E_ENABLE;

	/*Clear timer*/
	TIM8->CNT = 0;
	/*enable timer*/
	TIM8->BDTR |= (1U<<15);
	TIM8->CR1 |= CR1_CEN;
}

