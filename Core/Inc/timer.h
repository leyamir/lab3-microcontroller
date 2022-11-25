/*
 * timer.h
 *
 *  Created on: Nov 25, 2022
 *      Author: USER
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#define TIMER_CYCLE 10

int trafic_flag;
int trafic_counter;
int seg7_flag;
int seg7_counter;

int blink_counter;
int blink_flag;

void set_Trafic_Timer(int duration);
void Trafic_Timer_Run();

void set_seg7_led_timer(int duration);
void seg7_led_timer_run();

void set_blink_timer(int duration);
void blink_timer_run();

#endif /* INC_TIMER_H_ */
