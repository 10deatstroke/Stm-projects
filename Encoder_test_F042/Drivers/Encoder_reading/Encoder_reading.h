/*
 * Encoder_reading.h
 *
 *  Created on: Feb 3, 2023
 *      Author: OraMic_Terminal
 */

#ifndef ENCODER_READING_ENCODER_READING_H_
#define ENCODER_READING_ENCODER_READING_H_

#include "main.h"

struct encoder_input{
	bool encoder_a_input;
	bool encoder_b_input;
	bool prev_encoder_a_input;
	bool prev_encoder_b_input;
	uint8_t encoder_a_update_counter;
	uint8_t encoder_b_update_counter;
};

void Interrupt_reader(uint16_t);
void Input_2_change_mode();


#endif /* ENCODER_READING_ENCODER_READING_H_ */
