/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void read_slider();
void imu_setup();
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define col_1_Pin GPIO_PIN_0
#define col_1_GPIO_Port GPIOC
#define button_up_Pin GPIO_PIN_2
#define button_up_GPIO_Port GPIOC
#define button_up_EXTI_IRQn EXTI2_IRQn
#define button_down_Pin GPIO_PIN_3
#define button_down_GPIO_Port GPIOC
#define button_down_EXTI_IRQn EXTI3_IRQn
#define button_middle_Pin GPIO_PIN_0
#define button_middle_GPIO_Port GPIOA
#define button_middle_EXTI_IRQn EXTI0_IRQn
#define button_right_Pin GPIO_PIN_1
#define button_right_GPIO_Port GPIOA
#define button_right_EXTI_IRQn EXTI1_IRQn
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define button_left_Pin GPIO_PIN_4
#define button_left_GPIO_Port GPIOA
#define button_left_EXTI_IRQn EXTI4_IRQn
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define debug_1_Pin GPIO_PIN_6
#define debug_1_GPIO_Port GPIOA
#define debug_2_Pin GPIO_PIN_7
#define debug_2_GPIO_Port GPIOA
#define col_2_Pin GPIO_PIN_0
#define col_2_GPIO_Port GPIOB
#define row_7_Pin GPIO_PIN_1
#define row_7_GPIO_Port GPIOB
#define row_6_Pin GPIO_PIN_2
#define row_6_GPIO_Port GPIOB
#define row_5_Pin GPIO_PIN_11
#define row_5_GPIO_Port GPIOB
#define row_4_Pin GPIO_PIN_12
#define row_4_GPIO_Port GPIOB
#define row_1_Pin GPIO_PIN_6
#define row_1_GPIO_Port GPIOC
#define debug_3_Pin GPIO_PIN_7
#define debug_3_GPIO_Port GPIOC
#define row_0_Pin GPIO_PIN_8
#define row_0_GPIO_Port GPIOC
#define debug_4_Pin GPIO_PIN_9
#define debug_4_GPIO_Port GPIOA
#define col_0_Pin GPIO_PIN_10
#define col_0_GPIO_Port GPIOA
#define row_3_Pin GPIO_PIN_11
#define row_3_GPIO_Port GPIOA
#define row_2_Pin GPIO_PIN_12
#define row_2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define col_7_Pin GPIO_PIN_15
#define col_7_GPIO_Port GPIOA
#define col_3_Pin GPIO_PIN_10
#define col_3_GPIO_Port GPIOC
#define col_4_Pin GPIO_PIN_11
#define col_4_GPIO_Port GPIOC
#define col_5_Pin GPIO_PIN_12
#define col_5_GPIO_Port GPIOC
#define col_6_Pin GPIO_PIN_2
#define col_6_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
