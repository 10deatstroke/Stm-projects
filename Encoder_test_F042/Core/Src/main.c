/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define OUTPUT_LOOPED_IN	1

#define COMPLETE_ROTATION_COUNT		20.48

#define FIRST_PROFILE_MIN_PULSE 22
#define FIRST_PROFILE_MAX_PULSE 24

#define SECOND_PROFILE_MIN_PULSE 67
#define SECOND_PROFILE_MAX_PULSE 69

#define THIRD_PROFILE_MIN_PULSE 114
#define THIRD_PROFILE_MAX_PULSE 116

#define FOURTH_PROFILE_MIN_PULSE 164
#define FOURTH_PROFILE_MAX_PULSE 166

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t pulse_profile[4] = {40, 27, 21, 14};

uint32_t base_sec_counter;		// will be holding milliseconds, so result will be x1000 for seconds

// 	encoder input variables
uint8_t old_enc_state, new_enc_state;

int32_t input_counter, input_counter_display, output_counter_A, output_counter_B;

float rotation_position;	//	will hold current position in complete rotation
uint16_t rotation_count;	// will hold total rotations executed till now


#if OUTPUT_LOOPED_IN == 1
uint8_t generated_out_enc_old_state, generated_out_enc_new_state;
int32_t generated_output;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */
void Interrupt_reader(uint16_t);
void set_base_sec_timer(uint32_t time);
void reset_input_counter(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  set_base_sec_timer(HAL_GetTick());
  HAL_TIM_IC_Start_IT(&htim14, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim14);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t current_time = HAL_GetTick();

//	  timer and counter reset logic
	  if(current_time - base_sec_counter > 7000){
		  set_base_sec_timer(current_time);
		  reset_input_counter();
	  }
	  rotation_position = input_counter/(float)COMPLETE_ROTATION_COUNT;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 48-1;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 500-1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 250-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Out_Encoder_A_Pin|Out_Encoder_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Out_Encoder_A_Pin Out_Encoder_B_Pin */
  GPIO_InitStruct.Pin = Out_Encoder_A_Pin|Out_Encoder_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENCODER_B_Pin ENCODER_A_Pin */
  GPIO_InitStruct.Pin = ENCODER_B_Pin|ENCODER_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Temp_in_B_Pin Temp_in_A_Pin */
  GPIO_InitStruct.Pin = Temp_in_B_Pin|Temp_in_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */

void Interrupt_reader(uint16_t Gpio_pin) {
	/*
	 * states
	 * 	+---+----+
	 * 	|A	|	B|
	 * 	+---+----+
	 * 	|0	|	0|
	 * 	|0	|	1|
	 * 	|1	|	1|
	 * 	|1	|	0|
	 * 	|0	|	0|
	 * 	+---+----+
	 */
	if (Gpio_pin == ENCODER_A_Pin || Gpio_pin == ENCODER_B_Pin) {

		new_enc_state = HAL_GPIO_ReadPin(ENCODER_A_GPIO_Port, ENCODER_A_Pin) << 1 | HAL_GPIO_ReadPin(ENCODER_B_GPIO_Port, ENCODER_B_Pin);

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
		if((input_counter % pulse_profile[3]) == 0){
			input_counter++;
		}
		old_enc_state = new_enc_state;
		set_base_sec_timer(HAL_GetTick());
	}

#if	OUTPUT_LOOPED_IN == 1
	if (Gpio_pin == Temp_in_A_Pin || Gpio_pin == Temp_in_B_Pin) {

		generated_out_enc_new_state = HAL_GPIO_ReadPin(Temp_in_A_GPIO_Port, Temp_in_A_Pin) << 1 | HAL_GPIO_ReadPin(Temp_in_B_GPIO_Port, Temp_in_B_Pin);

		if (generated_out_enc_old_state == 0 && generated_out_enc_new_state == 1) {
			generated_output++;
		} else if (generated_out_enc_old_state == 1 && generated_out_enc_new_state == 3) {
			generated_output++;
		} else if (generated_out_enc_old_state == 3 && generated_out_enc_new_state == 2) {
			generated_output++;
		} else if (generated_out_enc_old_state == 2 && generated_out_enc_new_state == 0) {
			generated_output++;
		} else if (generated_out_enc_old_state == 0 && generated_out_enc_new_state == 2) {
			generated_output--;
		} else if (generated_out_enc_old_state == 2 && generated_out_enc_new_state == 3) {
			generated_output--;
		} else if (generated_out_enc_old_state == 3 && generated_out_enc_new_state == 1) {
			generated_output--;
		} else if (generated_out_enc_old_state == 1 && generated_out_enc_new_state == 0) {
			generated_output--;
		}
		generated_out_enc_old_state = generated_out_enc_new_state;
	}
#endif
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim14){
		if(output_counter_A < input_counter){
			HAL_GPIO_TogglePin(Out_Encoder_A_GPIO_Port, Out_Encoder_A_Pin);
			output_counter_A++;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim14){
		if(output_counter_B < input_counter){
			HAL_GPIO_TogglePin(Out_Encoder_B_GPIO_Port, Out_Encoder_B_Pin);
			output_counter_B++;
		}
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	Interrupt_reader(GPIO_Pin);
}

void set_base_sec_timer(uint32_t time){
	base_sec_counter = time;
}

void reset_input_counter(void){
	input_counter = 0;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
