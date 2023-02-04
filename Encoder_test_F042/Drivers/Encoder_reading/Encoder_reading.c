/*
 * Encoder_reading.c
 *
 *  Created on: Feb 3, 2023
 *      Author: OraMic_Terminal
 */

#include "main.h"
#include "Encoder_reading.h"

uint8_t old_enc_state = 0;
uint8_t new_enc_state = 0;

uint8_t updated_old_enc_state = 0;
uint8_t updated_new_enc_state = 0;

int32_t input_counter = 0;
int32_t generated_output = 0;

uint8_t current_profile = 0;

uint16_t profile_map[5] = {1,2,3,4,5};

struct encoder_input enc_input = { .encoder_a_input = 0, .encoder_b_input = 0,
		.prev_encoder_a_input = 0, .prev_encoder_b_input = 0 };

void Interrupt_reader(uint16_t Gpio_pin) {
	/*
	 * 	0	-	0
	 * 	0	-	1
	 * 	1	-	1
	 * 	1	-	0
	 * 	0	-	0
	 */
	if (Gpio_pin == ENCODER_A_Pin || Gpio_pin == ENCODER_B_Pin) {
		enc_input.encoder_a_input = HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin);
		enc_input.encoder_b_input = HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);
		if (enc_input.prev_encoder_a_input != enc_input.encoder_a_input){
			enc_input.prev_encoder_a_input = enc_input.encoder_a_input;
			enc_input.encoder_a_update_counter++;
			if(enc_input.encoder_a_update_counter >= profile_map[current_profile]){
			enc_input.encoder_a_update_counter = 0;
				HAL_GPIO_TogglePin(Out_Encoder_A_GPIO_Port, Out_Encoder_A_Pin);
			}
		}
		if (enc_input.prev_encoder_b_input != enc_input.encoder_b_input){
			enc_input.prev_encoder_b_input = enc_input.encoder_b_input;
			enc_input.encoder_b_update_counter++;
			if(enc_input.encoder_b_update_counter >= profile_map[current_profile]){
			enc_input.encoder_b_update_counter = 0;
				HAL_GPIO_TogglePin(Out_Encoder_B_GPIO_Port, Out_Encoder_B_Pin);
			}
		}
		new_enc_state = enc_input.encoder_a_input << 1 | enc_input.encoder_b_input;
		if (old_enc_state == 0 && new_enc_state == 1) {
			input_counter++;
		} else if (old_enc_state == 1 && new_enc_state == 3) {
			input_counter++;
		} else if (old_enc_state == 3 && new_enc_state == 2) {
			input_counter++;
		} else if (old_enc_state == 2 && new_enc_state == 0) {
			input_counter++;
		} else if (old_enc_state == 0 && new_enc_state == 2) {
			input_counter--;
		} else if (old_enc_state == 2 && new_enc_state == 3) {
			input_counter--;
		} else if (old_enc_state == 3 && new_enc_state == 1) {
			input_counter--;
		} else if (old_enc_state == 1 && new_enc_state == 0) {
			input_counter--;
		}
		old_enc_state = new_enc_state;
	}


	if (Gpio_pin == Temp_in_a_Pin || Gpio_pin == Temp_in_b_Pin) {
		updated_new_enc_state = HAL_GPIO_ReadPin(Temp_in_a_GPIO_Port, Temp_in_a_Pin) << 1 | HAL_GPIO_ReadPin(Temp_in_b_GPIO_Port, Temp_in_b_Pin);
		if (updated_old_enc_state == 0 && updated_new_enc_state == 1) {
			generated_output++;
		} else if (updated_old_enc_state == 1 && updated_new_enc_state == 3) {
			generated_output++;
		} else if (updated_old_enc_state == 3 && updated_new_enc_state == 2) {
			generated_output++;
		} else if (updated_old_enc_state == 2 && updated_new_enc_state == 0) {
			generated_output++;
		} else if (updated_old_enc_state == 0 && updated_new_enc_state == 2) {
			generated_output--;
		} else if (updated_old_enc_state == 2 && updated_new_enc_state == 3) {
			generated_output--;
		} else if (updated_old_enc_state == 3 && updated_new_enc_state == 1) {
			generated_output--;
		} else if (updated_old_enc_state == 1 && updated_new_enc_state == 0) {
			generated_output--;
		}
		updated_old_enc_state = updated_new_enc_state;
	}
}

void Input_2_change_mode() {

}
