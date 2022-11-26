/*
 * global.h
 *
 *  Created on: Nov 26, 2022
 *      Author: USER
 */

#ifndef INC_GLOBAL_H_
#define INC_GLOBAL_H_

int mode;
int red_count_down;
int green_count_down;
int yellow_count_down;

int current_state_trafic1;
int current_state_trafic2;
int trafic1_count_down;
int trafic2_count_down;

int seg7_led_buffer[4];
int seg7_led_order;

int duration;
int frequent;

void init_data();

#endif /* INC_GLOBAL_H_ */
