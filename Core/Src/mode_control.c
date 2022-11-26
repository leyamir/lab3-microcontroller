/*
 * mode_control.c
 *
 *  Created on: Nov 26, 2022
 *      Author: USER
 */
#include "mode_control.h"
#include "led.h"
#include "timer.h"
#include "global.h"
#include "button.h"

void run_normal_mode() {
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
void run_adjust_red_mode() {
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
void run_adjust_green_mode() {
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
void run_adjust_yellow_mode() {
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

void change_mode_state() {
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
}

void run_set_mode() {
	  mode = 1;
	  current_state_trafic1 = RED_STATE;
	  current_state_trafic2 = GREEN_STATE;
	  trafic1_count_down = red_count_down;
	  trafic2_count_down = green_count_down;
	  init_led();
	  set_seg7_led_timer(100);
	  set_Trafic_Timer(duration);
	  set_blink_timer(500);
}

