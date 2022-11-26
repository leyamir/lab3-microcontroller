/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN0_Pin GPIO_PIN_1
#define EN0_GPIO_Port GPIOA
#define EN1_Pin GPIO_PIN_2
#define EN1_GPIO_Port GPIOA
#define EN2_Pin GPIO_PIN_3
#define EN2_GPIO_Port GPIOA
#define EN3_Pin GPIO_PIN_4
#define EN3_GPIO_Port GPIOA
#define RED1_Pin GPIO_PIN_5
#define RED1_GPIO_Port GPIOA
#define YELLOW1_Pin GPIO_PIN_6
#define YELLOW1_GPIO_Port GPIOA
#define GREEN1_Pin GPIO_PIN_7
#define GREEN1_GPIO_Port GPIOA
#define a_1_Pin GPIO_PIN_0
#define a_1_GPIO_Port GPIOB
#define b_1_Pin GPIO_PIN_1
#define b_1_GPIO_Port GPIOB
#define c_1_Pin GPIO_PIN_2
#define c_1_GPIO_Port GPIOB
#define RED2_Pin GPIO_PIN_8
#define RED2_GPIO_Port GPIOA
#define YELLOW2_Pin GPIO_PIN_9
#define YELLOW2_GPIO_Port GPIOA
#define GREEN2_Pin GPIO_PIN_10
#define GREEN2_GPIO_Port GPIOA
#define MODE_Pin GPIO_PIN_11
#define MODE_GPIO_Port GPIOA
#define INC_Pin GPIO_PIN_12
#define INC_GPIO_Port GPIOA
#define SET_Pin GPIO_PIN_13
#define SET_GPIO_Port GPIOA
#define d_1_Pin GPIO_PIN_3
#define d_1_GPIO_Port GPIOB
#define e_1_Pin GPIO_PIN_4
#define e_1_GPIO_Port GPIOB
#define f_1_Pin GPIO_PIN_5
#define f_1_GPIO_Port GPIOB
#define g_1_Pin GPIO_PIN_6
#define g_1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define NORMAL_MODE 1
#define ADJUST_RED_LED 2
#define ADJUST_YELLOW_LED 3
#define ADJUST_GREEN_LED 4
#define RED_STATE 1
#define GREEN_STATE 2
#define YELLOW_STATE 3
#define DEFAULT_RED_COUNT_DOWN 5
#define DEFAULT_GREEN_COUNT_DOWN 3
#define DEFAULT_YELLOW_COUNT_DOWN 2
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
