/*
 * encoder.h
 *
 *  Created on: May 31, 2025
 *      Author: gaspr
 */

#ifndef ENCODER_H_
#define ENCODER_H_

typedef struct encoder_filter_t {
  uint16_t output_val;
  uint16_t current_val;
  uint16_t last_input_val;
  uint16_t direction;
} encoder_filter_t;

void init_encoder(void);
void encoder_callback(uint16_t captured_value, uint16_t direction_value);
void set_encoder_debug();
void clear_encoder_debug();

#endif /* ENCODER_H_ */
