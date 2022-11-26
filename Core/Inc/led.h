/*
 * led.h
 *
 *  Created on: Nov 26, 2022
 *      Author: USER
 */
#include "main.h"
#ifndef INC_LED_H_
#define INC_LED_H_

void display7SEG(int num, GPIO_TypeDef * GPIO_TYPE, uint16_t a_Pin, uint16_t b_Pin, uint16_t c_Pin, uint16_t d_Pin, uint16_t e_Pin, uint16_t f_Pin, uint16_t g_Pin);

void scan_seg7_led();
void blink();
void turn_off_all_led();
void init_led();
void show_red(int road);
void show_green(int road);
void show_yellow(int road);
void normal_led_buffer();
void adjust_mode_red_led_buffer();
void adjust_mode_green_led_buffer();
void adjust_mode_yellow_led_buffer();

#endif /* INC_LED_H_ */
