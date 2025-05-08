/*
 * clock.h
 *
 *  Created on: Apr 30, 2025
 *      Author: gaspr
 */

#ifndef CLOCK_H_
#define CLOCK_H_

#define SYSFREQUENCY 	72000000
#define APB1FREQ		SYSFREQUENCY/2
#define APB2FREQ		SYSFREQUENCY

void clock_init(void);

#endif /* CLOCK_H_ */
