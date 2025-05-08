#include "stm32f4xx.h"

#define TIM4EN 	(1U<<2)
#define CR1_CEN (1U<<0)
#define OC1M_TOGGLE (0b011<<4)
#define CC1E_ENABLE (1U<<0)

void tim4_output_compare(void)
{
	/*Enable Clock access to timer 4*/
	RCC->APB1ENR |= TIM4EN;
	/* set prescaler value*/
	TIM4->PSC = 0; //72000 - 1;   //72 000 000/72 000 = 10 000
	/*Set Auto reload value*/
	TIM4->ARR = 1000 - 1;  //10 000/10 000 = 1 Hz

	/*Set Output compare toggle mode*/
	TIM4->CCMR1 |= OC1M_TOGGLE;
	/*enable timer 4 Ch 1 in compare mode*/
	TIM4->CCER |= CC1E_ENABLE;

	/*Clear timer*/
	TIM4->CNT = 0;
	/*enable timer*/
	TIM4->CR1 |= CR1_CEN;

}


