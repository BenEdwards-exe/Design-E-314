/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "matrix.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

extern uint8_t ball_x_pos;
extern uint8_t ball_y_pos;

extern uint8_t bat_y_pos;
extern uint8_t bat_x_pos;

extern uint8_t selected_maze_num;
extern uint8_t display_matrix_ball_flag;
extern uint8_t display_matrix_end_flag;

extern uint8_t tennis_ball_velocity;
extern uint8_t tennis_ball_direction;

extern int imu_direction_to_move[2];
extern uint8_t imu_setup_flag;

uint32_t last_update_time = 0;
uint32_t maze_uart_last_update_time = 0;


// Status for matrix as finite state machine
extern enum Status matrixStatus;

// UART messages
uint8_t stdNum[10] = "$22738002\n";
uint8_t calibration[10] = "$1x______\n";
uint8_t maze[10] = "$3xxxxN__\n";
uint8_t tennis[10] = "$2xxxxxxN\n";

// I2C data
uint8_t i2cData[10];
HAL_StatusTypeDef res;

volatile int ax;
volatile int ay;
volatile int az;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

void calibration_sequence();

void light_up_corners();
void transmit_ball_position();
void transmit_maze_uart();
void transmit_tennis_uart();

void read_imu();


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Run the calibration sequence
void calibration_sequence() {

	// Light up the columns
	for (uint8_t col = 0; col < 8; ++col) {
		light_up_LED(col, 0);
		light_up_LED(col, 1);
		light_up_LED(col, 2);
		light_up_LED(col, 3);
		light_up_LED(col, 4);
		light_up_LED(col, 5);
		light_up_LED(col, 6);
		light_up_LED(col, 7);

		// Update calibration message and transmit it
		calibration[2] = col + 48;
		HAL_UART_Transmit(&huart2, calibration, 10, 50);

		HAL_Delay(1000);
		reset_all_ports(0);
	}


}

void light_up_corners() {
		reset_all_ports(0);
		light_up_LED(0, 0);
		light_up_LED(0, 7);
		light_up_LED(7, 7);
		light_up_LED(7, 0);
}

// Read the value of the slider through ADC and
// update the bat y position
void read_slider() {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 200);
	uint32_t slider_adc_val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	bat_y_pos = (uint8_t) ( 6 - slider_adc_val / 585.14 );
}

void transmit_ball_position() {
	maze[2] = ball_x_pos + 48;
	maze[3] = ball_y_pos + 48;
	HAL_UART_Transmit(&huart2, maze, 10, 50);
}

void transmit_maze_uart() {
	maze[2] = ball_x_pos + 48;
	maze[3] = ball_y_pos + 48;
	maze[4] = (display_matrix_ball_flag + 48);
	maze[5] = (display_matrix_end_flag + 48);
	HAL_UART_Transmit(&huart2, maze, 10, 50);
}

void transmit_tennis_uart() {
	tennis[2] = ball_x_pos + 48;
	tennis[3] = ball_x_pos + 48;
	tennis[4] = tennis_ball_velocity + 48;
	tennis[5] = tennis_ball_direction + 48;
	tennis[6] = bat_x_pos + 48;
	tennis[7] = bat_y_pos + 48;
	HAL_UART_Transmit(&huart2, tennis, 10, 50);
}

void imu_setup() {
	i2cData[0] = 0x20;
	i2cData[1] = 0x57;
	res = HAL_I2C_Master_Transmit(&hi2c1, 0x30, i2cData, 2, 10);
}

void read_imu() {
	i2cData[0] = 0xA8;
	res = HAL_I2C_Master_Transmit(&hi2c1, 0x30, i2cData, 1, 10);
	res = HAL_I2C_Master_Receive(&hi2c1, 0x30, i2cData, 4, 10);

	ax = *((int16_t*)i2cData);
	ay = *((int16_t*)(i2cData+2));
}
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
  MX_ADC1_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Transmit student number on start-up
  HAL_UART_Transmit(&huart2, stdNum, 10, 1000);

  imu_setup();

  // Set matrix to display calibration first
  matrixStatus = CALIBRATION;
  reset_all_ports(0);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Occurs every 100ms
	  if (HAL_GetTick() - last_update_time >= 100) {
		  // Update last_update_time
		  last_update_time = HAL_GetTick();

		  if (matrixStatus == CALIBRATION) {
			  calibration_sequence();
			  matrixStatus = CORNERS;
		  }
		  else if (matrixStatus == CORNERS) {
			  // Light up the four corners
			  light_up_corners();
		  }
		  /*
		  else if (matrixStatus == BALL) {
			  transmit_ball_position();
			  reset_all_ports(1);
			  light_up_LED(ball_x_pos, ball_y_pos);

			  // Update y position from adc
			  read_slider();
		  }
		  */
		  else if (matrixStatus == MAZE_SELECTION) {

		  }
		  else if (matrixStatus == MAZE) {
			  load_preset_maze(selected_maze_num);
		  }

		  else if (matrixStatus == TENNIS) {
			  // Tennis UART transmit
			  transmit_tennis_uart();
		  }

		  // IMU
		  read_imu();
		  update_imu_direction(ax, ay);
		  if (matrixStatus == TENNIS) {
			  move_bat_imu();
		  }
		  else if (matrixStatus == MAZE) {
			  move_ball_maze_imu();
		  }

	  }

	  // Maze UART Transmit (every 300ms)
	  if ((HAL_GetTick() - maze_uart_last_update_time >= 300) && (matrixStatus == MAZE)) {
		  transmit_maze_uart();
		  maze_uart_last_update_time = HAL_GetTick();
	  }









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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, col_1_Pin|row_1_Pin|debug_3_Pin|row_0_Pin 
                          |col_3_Pin|col_4_Pin|col_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|debug_1_Pin|debug_2_Pin|debug_4_Pin 
                          |col_0_Pin|row_3_Pin|row_2_Pin|col_7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, col_2_Pin|row_7_Pin|row_6_Pin|row_5_Pin 
                          |row_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(col_6_GPIO_Port, col_6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : col_1_Pin row_1_Pin debug_3_Pin row_0_Pin 
                           col_3_Pin col_4_Pin col_5_Pin */
  GPIO_InitStruct.Pin = col_1_Pin|row_1_Pin|debug_3_Pin|row_0_Pin 
                          |col_3_Pin|col_4_Pin|col_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : button_up_Pin button_down_Pin */
  GPIO_InitStruct.Pin = button_up_Pin|button_down_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : button_middle_Pin button_right_Pin button_left_Pin */
  GPIO_InitStruct.Pin = button_middle_Pin|button_right_Pin|button_left_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin debug_1_Pin debug_2_Pin debug_4_Pin 
                           col_0_Pin row_3_Pin row_2_Pin col_7_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|debug_1_Pin|debug_2_Pin|debug_4_Pin 
                          |col_0_Pin|row_3_Pin|row_2_Pin|col_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : col_2_Pin row_7_Pin row_6_Pin row_5_Pin 
                           row_4_Pin */
  GPIO_InitStruct.Pin = col_2_Pin|row_7_Pin|row_6_Pin|row_5_Pin 
                          |row_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : col_6_Pin */
  GPIO_InitStruct.Pin = col_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(col_6_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
