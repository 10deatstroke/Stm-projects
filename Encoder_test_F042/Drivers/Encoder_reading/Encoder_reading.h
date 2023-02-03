/*
 * Encoder_reading.h
 *
 *  Created on: Feb 3, 2023
 *      Author: OraMic_Terminal
 */

#ifndef ENCODER_READING_ENCODER_READING_H_
#define ENCODER_READING_ENCODER_READING_H_

#include "main.h"

/*
 * 0 - Interrupt based reading
 * 1 - Timer based reading
*/
#define ENCODER_READING_MODE	0


#if ENCODER_READING_MODE == 1
void Timer_read_start(TIM_HandleTypeDef*);
#else
void Interrupt_reader(uint16_t);
#endif

void Input_2_change_mode(){

}

#endif /* ENCODER_READING_ENCODER_READING_H_ */
