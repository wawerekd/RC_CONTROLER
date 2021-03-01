/*
 * buttonProcess.h
 *
 *  Created on: 24.01.2020
 *      Author: dwawerek
 */
#include "main.h"

#ifndef BUTTONPROCESS_H_
#define BUTTONPROCESS_H_

#define SHORT_PRESS   20   //*ms
#define MEDIUM_PRESS  500   //*ms
#define LONG_PRESS    2000  //*ms


#define NUM_OF_BUTTONS 3

typedef struct _Button_State {

	uint16_t tick_ms :11;
	uint16_t actual_state :1;
	uint16_t last_state :1;
	uint16_t short_press :1;
	uint16_t medium_press :1;
	uint16_t long_press :1;
	uint8_t long_press_debounce;


} Button_State;

Button_State buttons_state[NUM_OF_BUTTONS];

enum BUTTONS {
	ENCODER, JOY1, JOY2,
};

void clear_buttons_event(uint8_t button_number);
void process_buttons();
void read_buttons_state();

#endif /* BUTTONPROCESS_H_ */
