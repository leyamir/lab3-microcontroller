/*
 * led.c
 *
 *  Created on: Nov 26, 2022
 *      Author: USER
 */
#include "led.h"
#include "timer.h"
#include "main.h"
seg7_led_order = 0;

void normal_led_buffer() {
	  seg7_led_buffer[0] = trafic1_count_down / 10;
	  seg7_led_buffer[1] = trafic1_count_down % 10;
	  seg7_led_buffer[2] = trafic2_count_down / 10;
	  seg7_led_buffer[3] = trafic2_count_down % 10;
}

void adjust_mode_red_led_buffer() {
	  seg7_led_buffer[0] = red_count_down / 10;
	  seg7_led_buffer[1] = red_count_down % 10;
	  seg7_led_buffer[2] = 0;
	  seg7_led_buffer[3] = mode;
}

void adjust_mode_yellow_led_buffer() {
	  seg7_led_buffer[0] = yellow_count_down / 10;
	  seg7_led_buffer[1] = yellow_count_down % 10;
	  seg7_led_buffer[2] = 0;
	  seg7_led_buffer[3] = mode;
}

void adjust_mode_green_led_buffer() {
	  seg7_led_buffer[0] = green_count_down / 10;
	  seg7_led_buffer[1] = green_count_down % 10;
	  seg7_led_buffer[2] = 0;
	  seg7_led_buffer[3] = mode;
}

void show_red(int road) {
	if (road == 1) {
		  HAL_GPIO_WritePin(GPIOA, RED1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, YELLOW1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GREEN1_Pin, GPIO_PIN_SET);
	} else if (road == 2) {
		  HAL_GPIO_WritePin(GPIOA, RED2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, YELLOW2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GREEN2_Pin, GPIO_PIN_SET);
	}
}
void show_green(int road) {
	if (road == 1) {
		  HAL_GPIO_WritePin(GPIOA, RED1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, YELLOW1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GREEN1_Pin, GPIO_PIN_RESET);
	} else if (road == 2) {
		  HAL_GPIO_WritePin(GPIOA, RED2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, YELLOW2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GREEN2_Pin, GPIO_PIN_RESET);
	}
}
void show_yellow(int road) {
	if (road == 1) {
		  HAL_GPIO_WritePin(GPIOA, RED1_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, YELLOW1_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GREEN1_Pin, GPIO_PIN_SET);
	} else if (road == 2) {
		  HAL_GPIO_WritePin(GPIOA, RED2_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, YELLOW2_Pin, GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOA, GREEN2_Pin, GPIO_PIN_SET);
	}
}

void init_led() {
	  HAL_GPIO_WritePin(GPIOA, RED1_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, YELLOW1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, GREEN1_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, RED2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, YELLOW2_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, GREEN2_Pin, GPIO_PIN_RESET);
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

