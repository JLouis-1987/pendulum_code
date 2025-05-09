/*
 * timer.h
 *
 *  Created on: Apr 29, 2025
 *      Author: gaspr
 */

#ifndef TIMER_H_
#define TIMER_H_

void tim4_output_compare(void);
void tim8_output_compare(void);

#define SR_UIF (1U<<0)

#endif /* TIMER_H_ */
