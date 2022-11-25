/*
 * button.h
 *
 *  Created on: Nov 25, 2022
 *      Author: USER
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#define NORMAL_STATE GPIO_PIN_SET
#define PRESSED_STATE GPIO_PIN_RESET

void get_mode_input();
void get_inc_input();
void get_set_input();

int change_mode;
int inc_detect;
int save_all_change;

#endif /* INC_BUTTON_H_ */
