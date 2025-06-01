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
#include "encoder.h"

static help_str =	"\n\n*****Help Menu*****\n"
					"h - shows this menu\n"
					"w - pings IMU \n"
					"q - enable encoder\n"
					"a - disables encoder\n";

void process_commands(void){
	uint8_t input;
	input = rx_buffer_read();

	switch(input){
	case 'h':
		printf(help_str);
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
	case 'q':
		printf("Enabling Encoder Debug\n\r");
		set_encoder_debug();
		break;
	case 'a':
		printf("Disabling Encoder Debug\n\r");
		clear_encoder_debug();
		break;
	default:
		printf("CMD NOT VALID!!!\n\r");
		break;
	}

}
