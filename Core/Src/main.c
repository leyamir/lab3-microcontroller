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

void display7SEG(int num, GPIO_TypeDef * GPIO_TYPE, uint16_t a_Pin, uint16_t b_Pin, uint16_t c_Pin, uint16_t d_Pin, uint16_t e_Pin, uint16_t f_Pin, uint16_t g_Pin);
int seg7_led_buffer[4] = {0, 0, 0, 0};
int seg7_led_order = 0;
int mode = 1;
void scan_seg7_led();\
void blink();
void turn_off_all_led();
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

  int red_count_down = DEFAULT_RED_COUNT_DOWN;
  int green_count_down = DEFAULT_GREEN_COUNT_DOWN;
  int yellow_count_down = DEFAULT_YELLOW_COUNT_DOWN;

  int current_state_trafic1 = 1; // 1: red, 2: green, 3: yellow
  int current_state_trafic2 = 2;
  int trafic1_count_down = red_count_down;
  int trafic2_count_down = green_count_down;

  seg7_led_buffer[0] = trafic1_count_down / 10;
  seg7_led_buffer[1] = trafic1_count_down % 10;
  seg7_led_buffer[2] = trafic2_count_down / 10;
  seg7_led_buffer[3] = trafic2_count_down % 10;
  HAL_GPIO_WritePin(GPIOA, RED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, YELLOW1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GREEN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, RED2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, YELLOW2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GREEN2_Pin, GPIO_PIN_RESET);
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
		  case 1:
			  mode = 2;
			  break;
		  case 2:
		  	  mode = 3;
		  	  break;
		  case 3:
			  mode = 4;
			  break;
		  case 4:
			  mode = 2;
			  break;
		  default:
			  break;
		  }
		  change_mode = 0;
	  }
	  if (save_all_change == 1) {
		  mode = 1;
		  current_state_trafic1 = 1; // 1: red, 2: green, 3: yellow
		  current_state_trafic2 = 2;
		  trafic1_count_down = red_count_down;
		  trafic2_count_down = green_count_down;
		  HAL_GPIO_WritePin(GPIOA, RED1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, YELLOW1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GREEN1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, RED2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, YELLOW2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GREEN2_Pin, GPIO_PIN_RESET);
		  set_seg7_led_timer(100);
		  set_Trafic_Timer(duration);
		  set_blink_timer(500);
		  save_all_change = 0;
	  }
	  if (mode == 1) {
		  // TODO run system
		  seg7_led_buffer[0] = trafic1_count_down / 10;
		  seg7_led_buffer[1] = trafic1_count_down % 10;
		  seg7_led_buffer[2] = trafic2_count_down / 10;
		  seg7_led_buffer[3] = trafic2_count_down % 10;
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
		  if (trafic1_count_down == 0 && current_state_trafic1 == 1) {
			  HAL_GPIO_WritePin(GPIOA, RED1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, YELLOW1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, GREEN1_Pin, GPIO_PIN_RESET);
			  trafic1_count_down = green_count_down;
			  current_state_trafic1 = 2;
		  }
		  if (trafic1_count_down == 0 && current_state_trafic1 == 2) {
			  HAL_GPIO_WritePin(GPIOA, RED1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, YELLOW1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GREEN1_Pin, GPIO_PIN_SET);
			  trafic1_count_down = yellow_count_down;
			  current_state_trafic1 = 3;
		  }
		  if (trafic1_count_down == 0 && current_state_trafic1 == 3) {
			  HAL_GPIO_WritePin(GPIOA, RED1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, YELLOW1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, GREEN1_Pin, GPIO_PIN_SET);
			  trafic1_count_down = red_count_down;
			  current_state_trafic1 = 1;
		  }

		  if (trafic2_count_down == 0 && current_state_trafic2 == 1) {
			  HAL_GPIO_WritePin(GPIOA, RED2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, YELLOW2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, GREEN2_Pin, GPIO_PIN_RESET);
			  trafic2_count_down = green_count_down;
			  current_state_trafic2 = 2;
		  }
		  if (trafic2_count_down == 0 && current_state_trafic2 == 2) {
			  HAL_GPIO_WritePin(GPIOA, RED2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, YELLOW2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, GREEN2_Pin, GPIO_PIN_SET);
			  trafic2_count_down = yellow_count_down;
			  current_state_trafic2 = 3;
		  }
		  if (trafic2_count_down == 0 && current_state_trafic2 == 3) {
			  HAL_GPIO_WritePin(GPIOA, RED2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, YELLOW2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, GREEN2_Pin, GPIO_PIN_SET);
			  trafic2_count_down = red_count_down;
			  current_state_trafic2 = 1;
		  }
	  }
	  else if (mode == 2) {
		  // TODO adjust red
		  seg7_led_buffer[0] = red_count_down / 10;
		  seg7_led_buffer[1] = red_count_down % 10;
		  seg7_led_buffer[2] = 0;
		  seg7_led_buffer[3] = mode;
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
	  else if (mode == 3) {
		  // TODO adjust green
		  seg7_led_buffer[0] = yellow_count_down / 10;
		  seg7_led_buffer[1] = yellow_count_down % 10;
		  seg7_led_buffer[2] = 0;
		  seg7_led_buffer[3] = mode;
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
	  else if (mode == 4) {
		  // TODO adjust yellow
		  seg7_led_buffer[0] = green_count_down / 10;
		  seg7_led_buffer[1] = green_count_down % 10;
		  seg7_led_buffer[2] = 0;
		  seg7_led_buffer[3] = mode;
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
void display7SEG(int num, GPIO_TypeDef * GPIO_TYPE, uint16_t a_Pin, uint16_t b_Pin, uint16_t c_Pin, uint16_t d_Pin, uint16_t e_Pin, uint16_t f_Pin, uint16_t g_Pin) {
	switch (num){
		case 0:
			HAL_GPIO_WritePin(GPIO_TYPE, a_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, b_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, c_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, d_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, e_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, f_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, g_Pin, GPIO_PIN_SET);
			break;
		case 1:
			HAL_GPIO_WritePin(GPIO_TYPE, a_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, b_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, c_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, d_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, e_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, f_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, g_Pin, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(GPIO_TYPE, a_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, b_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, c_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, d_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, e_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, f_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, g_Pin, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(GPIO_TYPE, a_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, b_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, c_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, d_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, e_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, f_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, g_Pin, GPIO_PIN_RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIO_TYPE, a_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, b_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, c_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, d_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, e_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, f_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, g_Pin, GPIO_PIN_RESET);
			break;
		case 5:
			HAL_GPIO_WritePin(GPIO_TYPE, a_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, b_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, c_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, d_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, e_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, f_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, g_Pin, GPIO_PIN_RESET);
			break;
		case 6:
			HAL_GPIO_WritePin(GPIO_TYPE, a_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, b_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, c_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, d_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, e_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, f_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, g_Pin, GPIO_PIN_RESET);
			break;
		case 7:
			HAL_GPIO_WritePin(GPIO_TYPE, a_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, b_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, c_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, d_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, e_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, f_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, g_Pin, GPIO_PIN_SET);
			break;
		case 8:
			HAL_GPIO_WritePin(GPIO_TYPE, a_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, b_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, c_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, d_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, e_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, f_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, g_Pin, GPIO_PIN_RESET);
			break;
		case 9:
			HAL_GPIO_WritePin(GPIO_TYPE, a_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, b_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, c_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, d_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, e_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIO_TYPE, f_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIO_TYPE, g_Pin, GPIO_PIN_RESET);
			break;
		default:
			break;
	}
}

void scan_seg7_led() {
	  if (seg7_flag == 1) {
		 switch (seg7_led_order) {
		 case 0:
			  HAL_GPIO_WritePin(GPIOA, EN0_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, EN1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, EN2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, EN3_Pin, GPIO_PIN_SET);
			  display7SEG(seg7_led_buffer[seg7_led_order], GPIOB, a_1_Pin, b_1_Pin, c_1_Pin, d_1_Pin, e_1_Pin, f_1_Pin, g_1_Pin);
			  seg7_led_order = 1;
			  break;
		 case 1:
			  HAL_GPIO_WritePin(GPIOA, EN0_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, EN1_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, EN2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, EN3_Pin, GPIO_PIN_SET);
			  display7SEG(seg7_led_buffer[seg7_led_order], GPIOB, a_1_Pin, b_1_Pin, c_1_Pin, d_1_Pin, e_1_Pin, f_1_Pin, g_1_Pin);
			  seg7_led_order = 2;
			  break;
		 case 2:
			  HAL_GPIO_WritePin(GPIOA, EN0_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, EN1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, EN2_Pin, GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOA, EN3_Pin, GPIO_PIN_SET);
			  display7SEG(seg7_led_buffer[seg7_led_order], GPIOB, a_1_Pin, b_1_Pin, c_1_Pin, d_1_Pin, e_1_Pin, f_1_Pin, g_1_Pin);
			  seg7_led_order = 3;
			  break;
		 case 3:
			  HAL_GPIO_WritePin(GPIOA, EN0_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, EN1_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, EN2_Pin, GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOA, EN3_Pin, GPIO_PIN_RESET);
			  display7SEG(seg7_led_buffer[seg7_led_order], GPIOB, a_1_Pin, b_1_Pin, c_1_Pin, d_1_Pin, e_1_Pin, f_1_Pin, g_1_Pin);
			  seg7_led_order = 0;
			  break;
		 default:
			 break;
		 }
		  set_seg7_led_timer(100);
	  }
}
void turn_off_all_led() {
	HAL_GPIO_WritePin(GPIOA, RED1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, YELLOW1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GREEN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, RED2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, YELLOW2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GREEN2_Pin, GPIO_PIN_SET);
}
void blink() {
	HAL_GPIO_TogglePin(GPIOA, RED1_Pin);
	HAL_GPIO_TogglePin(GPIOA, RED2_Pin);
	HAL_GPIO_TogglePin(GPIOA, GREEN1_Pin);
	HAL_GPIO_TogglePin(GPIOA, GREEN2_Pin);
	HAL_GPIO_TogglePin(GPIOA, YELLOW1_Pin);
	HAL_GPIO_TogglePin(GPIOA, YELLOW2_Pin);
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
