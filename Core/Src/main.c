/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "timer.h"
#include "button.h"
#include "led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEFAULT_RED_COUNT_DOWN 5
#define DEFAULT_GREEN_COUNT_DOWN 3
#define DEFAULT_YELLOW_COUNT_DOWN 2
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
mode = 1;
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(& htim2);

  int duration = 500;
  int frequent = 1000 / duration;

  red_count_down = DEFAULT_RED_COUNT_DOWN;
  green_count_down = DEFAULT_GREEN_COUNT_DOWN;
  yellow_count_down = DEFAULT_YELLOW_COUNT_DOWN;

  current_state_trafic1 = 1; // 1: red, 2: green, 3: yellow
  current_state_trafic2 = 2;
  trafic1_count_down = red_count_down;
  trafic2_count_down = green_count_down;

  seg7_led_buffer[0] = trafic1_count_down / 10;
  seg7_led_buffer[1] = trafic1_count_down % 10;
  seg7_led_buffer[2] = trafic2_count_down / 10;
  seg7_led_buffer[3] = trafic2_count_down % 10;
  init_led();
  set_seg7_led_timer(100);
  set_Trafic_Timer(duration);
  set_blink_timer(500);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (change_mode == 1) {
		  turn_off_all_led();
		  switch (mode) {
		  case NORMAL_MODE:
			  mode = 2;
			  break;
		  case ADJUST_RED_LED:
		  	  mode = 3;
		  	  break;
		  case ADJUST_YELLOW_LED:
			  mode = 4;
			  break;
		  case ADJUST_GREEN_LED:
			  mode = 2;
			  break;
		  default:
			  break;
		  }
		  change_mode = 0;
	  }
	  if (save_all_change == 1) {
		  mode = 1;
		  current_state_trafic1 = RED_STATE;
		  current_state_trafic2 = GREEN_STATE;
		  trafic1_count_down = red_count_down;
		  trafic2_count_down = green_count_down;
		  init_led();
		  set_seg7_led_timer(100);
		  set_Trafic_Timer(duration);
		  set_blink_timer(500);
		  save_all_change = 0;
	  }
	  if (mode == NORMAL_MODE) {
		  // TODO run system
		  normal_led_buffer();
		  scan_seg7_led();
		  if (trafic_flag == 1) {
			  if (frequent > 0) {
				  frequent--;
				  if (frequent == 0) {
					  trafic1_count_down--;
					  trafic2_count_down--;
					  frequent = 1000 / duration;
				  }
			  }
			  set_Trafic_Timer(duration);
		  }
		  if (trafic1_count_down == 0 && current_state_trafic1 == RED_STATE) {
			  show_green(1);
			  trafic1_count_down = green_count_down;
			  current_state_trafic1 = GREEN_STATE;
		  }
		  if (trafic1_count_down == 0 && current_state_trafic1 == GREEN_STATE) {
			  show_yellow(1);
			  trafic1_count_down = yellow_count_down;
			  current_state_trafic1 = YELLOW_STATE;
		  }
		  if (trafic1_count_down == 0 && current_state_trafic1 == YELLOW_STATE) {
			  show_red(1);
			  trafic1_count_down = red_count_down;
			  current_state_trafic1 = RED_STATE;
		  }

		  if (trafic2_count_down == 0 && current_state_trafic2 == RED_STATE) {
			  show_green(2);
			  trafic2_count_down = green_count_down;
			  current_state_trafic2 = GREEN_STATE;
		  }
		  if (trafic2_count_down == 0 && current_state_trafic2 == GREEN_STATE) {
			  show_yellow(2);
			  trafic2_count_down = yellow_count_down;
			  current_state_trafic2 = YELLOW_STATE;
		  }
		  if (trafic2_count_down == 0 && current_state_trafic2 == YELLOW_STATE) {
			  show_red(2);
			  trafic2_count_down = red_count_down;
			  current_state_trafic2 = RED_STATE;
		  }
	  }
	  else if (mode == ADJUST_RED_LED) {
		  // TODO adjust red
		  adjust_mode_red_led_buffer();
		  scan_seg7_led();
		  if (blink_flag == 1) {
			  blink();
			  set_blink_timer(500);
		  }
		  if (inc_detect == 1) {
			  red_count_down += 1;
			  if (red_count_down == 100) {
				  red_count_down = DEFAULT_RED_COUNT_DOWN;
			  }
			  inc_detect = 0;
		  }
	  }
	  else if (mode == ADJUST_YELLOW_LED) {
		  // TODO adjust green
		  adjust_mode_yellow_led_buffer();
		  scan_seg7_led();
		  if (blink_flag == 1) {
			  blink();
			  set_blink_timer(500);
		  }
		  if (inc_detect == 1) {
			  yellow_count_down += 1;
			  if (yellow_count_down == 100) {
				  yellow_count_down = DEFAULT_YELLOW_COUNT_DOWN;
			  }
			  inc_detect = 0;
		  }
	  }
	  else if (mode == ADJUST_GREEN_LED) {
		  // TODO adjust yellow
		  adjust_mode_green_led_buffer();
		  scan_seg7_led();
		  if (blink_flag == 1) {
			  blink();
			  set_blink_timer(500);
		  }
		  if (inc_detect == 1) {
			  green_count_down += 1;
			  if (green_count_down == 100) {
				  green_count_down = DEFAULT_GREEN_COUNT_DOWN;
			  }
			  inc_detect = 0;
		  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN0_Pin|EN1_Pin|EN2_Pin|EN3_Pin
                          |RED1_Pin|YELLOW1_Pin|GREEN1_Pin|RED2_Pin
                          |YELLOW2_Pin|GREEN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, a_1_Pin|b_1_Pin|c_1_Pin|d_1_Pin
                          |e_1_Pin|f_1_Pin|g_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN0_Pin EN1_Pin EN2_Pin EN3_Pin
                           RED1_Pin YELLOW1_Pin GREEN1_Pin RED2_Pin
                           YELLOW2_Pin GREEN2_Pin */
  GPIO_InitStruct.Pin = EN0_Pin|EN1_Pin|EN2_Pin|EN3_Pin
                          |RED1_Pin|YELLOW1_Pin|GREEN1_Pin|RED2_Pin
                          |YELLOW2_Pin|GREEN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : a_1_Pin b_1_Pin c_1_Pin d_1_Pin
                           e_1_Pin f_1_Pin g_1_Pin */
  GPIO_InitStruct.Pin = a_1_Pin|b_1_Pin|c_1_Pin|d_1_Pin
                          |e_1_Pin|f_1_Pin|g_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MODE_Pin INC_Pin SET_Pin */
  GPIO_InitStruct.Pin = MODE_Pin|INC_Pin|SET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim) {
	Trafic_Timer_Run();
	seg7_led_timer_run();
	blink_timer_run();
	get_mode_input();
	get_inc_input();
	get_set_input();
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
