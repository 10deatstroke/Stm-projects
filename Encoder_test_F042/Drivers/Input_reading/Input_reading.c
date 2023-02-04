/*
 * Input_reading.c
 *
 *  Created on: Feb 4, 2023
 *      Author: OraMic_Terminal
 */

#include "main.h"
#include "Input_reading.h"
#include "Encoder_reading.h"
#include "Output_reading.h"

extern int32_t input_counter;
extern int32_t generated_output;

void Input_reading(){
	 if(HAL_GPIO_ReadPin(Button_in_GPIO_Port, Button_in_Pin) == GPIO_PIN_RESET){
		 HAL_Delay(20);
		 if(HAL_GPIO_ReadPin(Button_in_GPIO_Port, Button_in_Pin) == GPIO_PIN_RESET){
			 input_counter = 0;
			 generated_output = 0;
		 }
	 }
}

