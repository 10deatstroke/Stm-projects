/*
 * Encoder_reading.c
 *
 *  Created on: Feb 3, 2023
 *      Author: OraMic_Terminal
 */

#include "Encoder_reading.h"

#if ENCODER_READING_MODE == 1
void Timer_read_start(TIM_HandleTypeDef *htim){
  HAL_TIM_Encoder_Start_IT(htim, TIM_CHANNEL_ALL);
}
#endif

void Interrupt_reader(uint16_t Gpio_pin){
	/*
	 * 	0	-	0
	 * 	0	-	1
	 * 	1	-	1
	 * 	1	-	0
	 * 	0	-	0
	 */
	if(GPIO_Pin == ENCODER_A_Pin || GPIO_Pin == ENCODER_B_Pin){
		new_enc_state = HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin) << 1| HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);
		if(old_enc_state == 0 && new_enc_state == 1){
			counter++;
		}else if(old_enc_state == 1 && new_enc_state == 3){
			counter++;
		}else if(old_enc_state == 3 && new_enc_state == 2){
			counter++;
		}else if(old_enc_state == 2 && new_enc_state == 0){
			counter++;
		}else if(old_enc_state == 0 && new_enc_state == 2){
			counter--;
		}else if(old_enc_state == 2 && new_enc_state == 3){
			counter--;
		}else if(old_enc_state == 3 && new_enc_state == 1){
			counter--;
		}else if(old_enc_state == 1 && new_enc_state == 0){
			counter--;
		}
		old_enc_state = new_enc_state;
	}
}
