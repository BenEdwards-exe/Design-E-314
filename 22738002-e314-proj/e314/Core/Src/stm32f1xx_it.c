/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "matrix.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
#define MAX_X 7
#define MIN_X 0
#define MAX_Y 7
#define MIN_X 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

extern uint8_t ball_x_pos;
extern uint8_t ball_y_pos;
extern enum Status matrixStatus;
extern uint8_t column_to_display;
extern uint8_t display_matrix_ball_flag; // 1 to display; 0 to not
extern uint8_t display_matrix_end_flag; // 1 to display; 0 to not
extern int selected_maze_num;
extern uint8_t should_load_num_flag;
extern uint8_t should_load_maze_flag;

extern uint16_t tennis_ball_timeout_time;
extern uint8_t tennis_ball_velocity;
extern uint8_t tennis_ball_hit_counter;
extern uint8_t tennis_ball_direction;
extern uint8_t bat_x_pos;
extern uint8_t bat_y_pos;

volatile uint32_t button_left_time = 0;
volatile uint32_t button_right_time = 0;
volatile uint32_t button_middle_time = 0;
volatile uint32_t button_up_time = 0;
volatile uint32_t button_down_time = 0;
volatile uint32_t ball_matrix_time = 0;
volatile uint32_t ball_maze_move_time = 0;
volatile uint32_t end_matrix_time = 0;
volatile uint32_t tennis_ball_move_time = 0;
volatile uint32_t bat_move_time = 0;

extern uint8_t imu_setup_flag;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	if (matrixStatus == MAZE_SELECTION) {
		// Update debug led
		light_debug_led(selected_maze_num, 1);
		// Display maze number on matrix
		load_preset_num_to_display(selected_maze_num);
		display_matrix();
	}
	else if (matrixStatus == MAZE) {
		// Update matrix ball display flag every 300ms
		if (HAL_GetTick() - ball_matrix_time >= 300) {
			display_matrix_ball_flag = !display_matrix_ball_flag;
			ball_matrix_time = HAL_GetTick();
		}
		// Update matrix end display flag every 100ms
		if (HAL_GetTick() - end_matrix_time >= 100) {
			display_matrix_end_flag = !display_matrix_end_flag;
			end_matrix_time = HAL_GetTick();
		}
		display_matrix();
		light_debug_led(selected_maze_num, 1); // Keep light on for selected maze
	}
	else if (matrixStatus == TENNIS) {
		display_matrix();
		if (HAL_GetTick() - tennis_ball_move_time >= (tennis_ball_timeout_time - ( 50*(tennis_ball_velocity - 1) ) ) ) {
			move_tennis_ball();
			tennis_ball_move_time = HAL_GetTick();
		}
	}


  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

	// middle
	if (HAL_GetTick() - button_middle_time >= 150) {
		// Update time
		button_middle_time = HAL_GetTick();
		// Change matrix status
		if (matrixStatus == MAZE_SELECTION) { // Maze has been selected
			light_debug_led(selected_maze_num, 1); // To keep debug on
			matrixStatus = MAZE; // Change to MAZE
			display_matrix_ball_flag = 1; // Reset flag
			display_matrix_end_flag = 1; // Reset flag
			end_matrix_time = HAL_GetTick(); // Reset timer counter
			ball_matrix_time = HAL_GetTick(); // Reset timer counter
			//imu_setup_flag = 1;
		}
		else if (matrixStatus == CORNERS) {
			matrixStatus = TENNIS;
			ball_x_pos = 7;
			ball_y_pos = 4;
			bat_x_pos = 0;
			bat_y_pos = 4;
			ball_matrix_time = HAL_GetTick();
			display_matrix_ball_flag = 1;
			tennis_ball_move_time = HAL_GetTick();
			//imu_setup_flag = 1;
		}
		else if (matrixStatus == MAZE) {
			should_load_maze_flag = 1;
			reset_debug_leds();
			matrixStatus = CORNERS;
		}
		else if (matrixStatus == TENNIS) {
			matrixStatus = CORNERS;
			tennis_ball_velocity = 1;
			tennis_ball_hit_counter = 0;
			tennis_ball_direction = 0;
		}

	}

  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles EXTI line1 interrupt.
  */
void EXTI1_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI1_IRQn 0 */
	// right
	if (HAL_GetTick() - button_right_time >= 100) {

		// Update time
		button_right_time = HAL_GetTick();

		// Move MAZE ball right
		if (matrixStatus == MAZE) {
			if (checkMazeBallMove(1, 0)) {
				++ball_x_pos;
				ball_maze_move_time = HAL_GetTick();
			}
		}
		// Move TENNIS bat right
		else if (matrixStatus == TENNIS) {
			if (checkBatMove(1, 0)) {
				++bat_x_pos;
				bat_move_time = HAL_GetTick();
			}
		}
		else if (ball_x_pos < MAX_X && matrixStatus == BALL) {
			++ball_x_pos;
		}
	}

  /* USER CODE END EXTI1_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
  /* USER CODE BEGIN EXTI1_IRQn 1 */

  /* USER CODE END EXTI1_IRQn 1 */
}

/**
  * @brief This function handles EXTI line2 interrupt.
  */
void EXTI2_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI2_IRQn 0 */
	// up
	if (HAL_GetTick() - button_up_time >= 100) {
		// Update time
		button_up_time = HAL_GetTick();
		if (matrixStatus == MAZE_SELECTION) {
			if (selected_maze_num < 4) {
				++selected_maze_num;
			}
			else {
				selected_maze_num = 1;
			}
			should_load_num_flag = 1;
		}
		// Move MAZE ball up
		else if (matrixStatus == MAZE) {
			if (checkMazeBallMove(0, -1)) {
				--ball_y_pos;
				ball_maze_move_time = HAL_GetTick();
			}
		}
		// Move TENNIS bat up
		else if (matrixStatus == TENNIS) {
			if (checkBatMove(0, -1)) {
				--bat_y_pos;
				bat_move_time = HAL_GetTick();
			}
		}
	}

  /* USER CODE END EXTI2_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
  /* USER CODE BEGIN EXTI2_IRQn 1 */

  /* USER CODE END EXTI2_IRQn 1 */
}

/**
  * @brief This function handles EXTI line3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
	// down
	if (HAL_GetTick() - button_down_time >= 100) {
		// Update time
		button_down_time = HAL_GetTick();
		if (matrixStatus == MAZE_SELECTION) {
			if (selected_maze_num > 1) {
				--selected_maze_num;
			}
			else {
				selected_maze_num = 4;
			}
			should_load_num_flag = 1;
		}
		// Move MAZE ball down
		else if (matrixStatus == MAZE) {
			if (checkMazeBallMove(0, 1)) {
				++ball_y_pos;
				ball_maze_move_time = HAL_GetTick();
			}
		}
		// Move TENNIS bat down
		else if (matrixStatus == TENNIS) {
			if (checkBatMove(0, 1)) {
				++bat_y_pos;
				bat_move_time = HAL_GetTick();
			}
		}
	}

  /* USER CODE END EXTI3_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles EXTI line4 interrupt.
  */
void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
	// left
	if (HAL_GetTick() - button_left_time >= 100) {
		// Update time
		button_left_time = HAL_GetTick();

		// If in CORNERS mode, change to MAZE_SELECT mode
		if (matrixStatus == CORNERS) {
			matrixStatus = MAZE_SELECTION;
			selected_maze_num = 1;
			should_load_num_flag = 1;
		}

		// If in MAZE_SELECT, change to CORNERS
		else if (matrixStatus == MAZE_SELECTION) {
			matrixStatus = CORNERS;
			reset_debug_leds();
		}

		// Move MAZE ball left
		else if (matrixStatus == MAZE) {
			if (checkMazeBallMove(-1, 0)) {
				--ball_x_pos;
				ball_maze_move_time = HAL_GetTick();
			}
		}

		// Move TENNIS bat left
		else if (matrixStatus == TENNIS) {
			if (checkBatMove(-1, 0)) {
				--bat_x_pos;
				bat_move_time = HAL_GetTick();
			}
		}

		// Move ball left
		else if (ball_x_pos > MIN_X && matrixStatus == BALL) {
			--ball_x_pos;
	}
	}
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  /* USER CODE BEGIN EXTI4_IRQn 1 */

  /* USER CODE END EXTI4_IRQn 1 */
}

/**
  * @brief This function handles EXTI line[15:10] interrupts.
  */
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
