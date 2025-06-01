/*
 * encoder.c
 *
 *  Created on: May 31, 2025
 *      Author: gaspr
 */


#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "stm32f4xx.h"
#include "encoder.h"

#define ALTFUN_AF2			(0b0010)
#define ALTFUN_4_12_MASK	0x000f0000
#define ALTFUN_5_13_MASK	0x00f00000
#define CCMR_CC1S_TI1 		0b01
#define CCMR_CC1S_TI1_MASK	0b01
#define CCMR_CC1S_IC1F 		0b0100
#define CCMR_CC1S_IC1F_MASK	0x0f
/*
 * Encoder Input TIM3
 * 	PB4 --> Ch1
 * 	PB5 --> Ch2
 */

void filter_encoder(encoder_filter_t* encoder_filter, uint16_t capture_val, uint16_t direction_value,
		bool encoder_debug_active);
void init_encoder_filter(encoder_filter_t* encoder_filter);


static encoder_filter_t encoder_filter = {0U};
static bool encoder_debug_active = false;

void init_encoder(void)
{
	/*
	 * Function: init_encoder
	 * ----------------------
	 * This function initialize the tim3 peripheral using Pin PB4 in an input capture mode
	 * and PB5 as just a GPIO input for direction. The function also enable the interrupt
	 * and initialized the encoder input filter.
	 * Assumes the APB1 clock frequency is 36 MHz
	 *
	 * Parameters:
	 *   None
	 *
	 * Returns:
	 *   None
	 */
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	//Set PB4 as alternate mode
	GPIOB->MODER &=~GPIO_MODER_MODE4_0;
	GPIOB->MODER |= GPIO_MODER_MODE4_1;

	//Set PB4 alternate mode to AF2 (Tim 3 CH 1)
	GPIOB->AFR[0]  = (GPIOB->AFR[0]  & ~ALTFUN_4_12_MASK) | (ALTFUN_AF2<<16);

	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

	//Sets timer frequency to 72kHz (Timer F = 2x APB)
	TIM3->PSC = 1000 - 1;

	// Set TIM3 to capture on CH1 (CC1S = 01)
	TIM3->CCMR1 |= TIM_CCMR1_CC1S_0;

	//TIM3->CCMR1 = (TIM3->CCMR1 & ~CCMR_CC1S_IC1F_MASK) | (CCMR_CC1S_IC1F);
	// Enable capture on rising edge
	TIM3->CCER |= TIM_CCER_CC1E;
	// Enable interrupt on capture event
	TIM3->DIER |= TIM_DIER_CC1IE;

	// Start TIM3
	TIM3->CR1 |= TIM_CR1_CEN;

	init_encoder_filter(&encoder_filter);

	NVIC_EnableIRQ(TIM3_IRQn);
}

void encoder_callback(uint16_t captured_value, uint16_t direction_value)
{
	/*
	 * Function: encoder_callback
	 * ----------------------
	 * This function is used when the interrupt is fired as the call back to this file.
	 *
	 * Parameters:
	 *   captured_value (uint16_t) - counts on the timer when the input capture pin was triggered
	 *   direction_value (unit16_t) - value of the GPIO input register
	 *
	 * Returns:
	 *   None
	 */
	filter_encoder(&encoder_filter, captured_value, direction_value, encoder_debug_active);
}

void init_encoder_filter(encoder_filter_t* encoder_filter) {
	/*
	 * Function: init_encoder_filter
	 * ----------------------
	 * This function initializes the encoder filter
	 *
	 * Parameters:
	 *   encoder_filter (struct) - structure with the encoder_filter parameters and data
	 *
	 * Returns:
	 *   None
	 */
	encoder_filter->output_val = 0;
	encoder_filter->current_val = 0;
	encoder_filter->last_input_val = 0;
	encoder_filter->direction = 0;
}

void filter_encoder(encoder_filter_t* encoder_filter, uint16_t capture_val, uint16_t direction_value,
		bool encoder_debug_active)
{
	/*
	 * Function: filter_encoder
	 * ----------------------
	 * This takes in the values from the interrupt call back and processes them through the encoder filter
	 * and load the data into the encoder structure. This will call a printf if the encoder_debug_active flag
	 * is set to true.
	 *
	 * Note: this function is called as part of an interrupt routine so it needs to stay fast.
	 *
	 * Parameters:
	 * 	 encoder_filter (structure) - structure with the encoder_filter parameters and data
	 *   capture_val (uint16_t) - counts on the timer when the input capture pin was triggered
	 *   direction_value (unit16_t) - value of the GPIO input register
	 *   encoder_debug_active (bool) - flag that enables the printf in this function for debug purposes
	 *
	 * Returns:
	 *   None
	 */
	encoder_filter->last_input_val = encoder_filter->current_val;
	encoder_filter->current_val = capture_val;
	encoder_filter->direction = direction_value;

	//Case where counter value has rolled over
	if(encoder_filter->last_input_val > encoder_filter->current_val)
	{
		encoder_filter->output_val = (65535 - encoder_filter->last_input_val) + encoder_filter->current_val;
	}
	else
	{
		encoder_filter->output_val =  encoder_filter->current_val - encoder_filter->last_input_val;
	}
	/* Wheel RPM 	= DC Motor RPS * 60/19
	 * 				= Encoder Freq/16 * 60/19
	 * 				= 72000/Counts *60/(19*16)
	 * 				= 72000*60/(19*16) / Counts
	 * 				= 1421.05/counts
	 */
	if(encoder_debug_active){
		printf("e, %u, %u\n", encoder_filter->output_val, encoder_filter->direction);
	}
}

void set_encoder_debug()
{
	/*
	 * Function: set_encoder_debug
	 * ----------------------
	 * Sets the encoder_debug_active to true.
	 *
	 * Parameters:
	 * 	 None
	 *
	 * Returns:
	 *   None
	 */
	encoder_debug_active = true;
}

void clear_encoder_debug()
{
	/*
	 * Function: clear_encoder_debug
	 * ----------------------
	 * Sets the encoder_debug_active to false.
	 *
	 * Parameters:
	 * 	 None
	 *
	 * Returns:
	 *   None
	 */
	encoder_debug_active = false;
}

