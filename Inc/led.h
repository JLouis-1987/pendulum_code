/*
 * led.h
 *
 *  Created on: Apr 29, 2025
 *      Author: gaspr
 */

#ifndef LED_H_
#define LED_H_


#include <stdint.h>
#include "stm32f4xx.h"

void init_leds(void);
void cycle_leds(void);

#endif /* LED_H_ */
