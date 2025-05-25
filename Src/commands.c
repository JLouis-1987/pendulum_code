/*
 * commands.c
 *
 *  Created on: May 14, 2025
 *      Author: gaspr
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "uart.h"
#include "clock.h"
#include "utils/ringbuffer.h"
#include "lsm6dso.h"

void process_commands(void){
	uint8_t input;
	input = rx_buffer_read();

	switch(input){
	case 'h':
		printf("Hello There!\n\r");
		break;
	case 'w':
		if(check_IMU() == true){
			printf("IMU is online\n");
		}
		else{
			/*TODO - Code will freeze waiting for unresponsive IMU, need to add time out*/
			printf("IMU is offline\n");
		}
		break;
	default:
		printf("CMD NOT VALID!!!\n\r");
		break;
	}

}
