/*
 * global.c
 *
 *  Created on: Nov 26, 2022
 *      Author: USER
 */

#include "global.h"
#include "main.h"

void init_data() {
	duration = 500;
	frequent = 1000 / duration;
	mode = 1;
	red_count_down = DEFAULT_RED_COUNT_DOWN;
	green_count_down = DEFAULT_GREEN_COUNT_DOWN;
	yellow_count_down = DEFAULT_YELLOW_COUNT_DOWN;

	current_state_trafic1 = RED_STATE;
	current_state_trafic2 = GREEN_STATE;
	trafic1_count_down = red_count_down;
	trafic2_count_down = green_count_down;

	seg7_led_buffer[0] = trafic1_count_down / 10;
	seg7_led_buffer[1] = trafic1_count_down % 10;
	seg7_led_buffer[2] = trafic2_count_down / 10;
	seg7_led_buffer[3] = trafic2_count_down % 10;
}

