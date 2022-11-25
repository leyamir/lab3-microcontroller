/*
 * timer.c
 *
 *  Created on: Nov 25, 2022
 *      Author: USER
 */
#include "timer.h"

trafic_flag = 0;
trafic_counter = 0;
seg7_flag = 0;
seg7_counter = 0;

void set_Trafic_Timer(int duration) {
	trafic_counter = duration / TIMER_CYCLE;
	trafic_flag = 0;
}

void Trafic_Timer_Run() {
	if (trafic_counter > 0) {
		trafic_counter--;
		if (trafic_counter == 0) {
			trafic_flag = 1;
		}
	}
}

void set_seg7_led_timer(int duration) {
	seg7_counter = duration / TIMER_CYCLE;
	seg7_flag = 0;
}
void seg7_led_timer_run() {
	if (seg7_counter > 0) {
		seg7_counter--;
		if (seg7_counter == 0) {
			seg7_flag = 1;
		}
	}
}

void set_blink_timer(int duration) {
	blink_counter = duration / TIMER_CYCLE;
	blink_flag = 0;
}
void blink_timer_run() {
	if (blink_counter > 0) {
		blink_counter--;
		if (blink_counter == 0) {
			blink_flag = 1;
		}
	}
}

