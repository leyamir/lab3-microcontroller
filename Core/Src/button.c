/*
 * button.c
 *
 *  Created on: Nov 25, 2022
 *      Author: USER
 */

#include "button.h"
#include "main.h"

int mode_reg_0 = NORMAL_STATE;
int mode_reg_1 = NORMAL_STATE;
int mode_reg_2 = NORMAL_STATE;

int mode_reg_3 = NORMAL_STATE;

change_mode = 0;

void get_mode_input() {
	mode_reg_0 = mode_reg_1;
	mode_reg_1 = mode_reg_2;
	mode_reg_2 = HAL_GPIO_ReadPin(GPIOA, MODE_Pin);
	if ((mode_reg_0 == mode_reg_1) && (mode_reg_1 == mode_reg_2)) {
		if (mode_reg_3 != mode_reg_2) {
			mode_reg_3 = mode_reg_2;
			if (mode_reg_2 == PRESSED_STATE) {
				// TODO change mode
				change_mode = 1;
			}
		}
	}
}

int inc_reg_0 = NORMAL_STATE;
int inc_reg_1 = NORMAL_STATE;
int inc_reg_2 = NORMAL_STATE;

int inc_reg_3 = NORMAL_STATE;

inc_detect = 0;

void get_inc_input() {
	inc_reg_0 = inc_reg_1;
	inc_reg_1 = inc_reg_2;
	inc_reg_2 = HAL_GPIO_ReadPin(GPIOA, INC_Pin);
	if ((inc_reg_0 == inc_reg_1) && (inc_reg_1 == inc_reg_2)) {
		if (inc_reg_3 != inc_reg_2) {
			inc_reg_3 = inc_reg_2;
			if (inc_reg_2 == PRESSED_STATE) {
				// TODO
				inc_detect = 1;
			}
		}
	}
}

int set_reg_0 = NORMAL_STATE;
int set_reg_1 = NORMAL_STATE;
int set_reg_2 = NORMAL_STATE;

int set_reg_3 = NORMAL_STATE;

save_all_change = 0;

void get_set_input() {
	set_reg_0 = set_reg_1;
	set_reg_1 = set_reg_2;
	set_reg_2 = HAL_GPIO_ReadPin(GPIOA, SET_Pin);
	if ((set_reg_0 == set_reg_1) && (set_reg_1 == set_reg_2)) {
		if (set_reg_3 != set_reg_2) {
			set_reg_3 = set_reg_2;
			if (set_reg_2 == PRESSED_STATE) {
				// TODO
				save_all_change = 1;
			}
		}
	}
}
