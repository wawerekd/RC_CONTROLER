/*
 * buttonProcess.c
 *
 *  Created on: 24.01.2020
 *      Author: dwawerek
 */

#include "buttonProcess.h"
#include "main.h"
#include <stdio.h>

//extern uint8_t buttons_states[NUM_OF_BUTTONS];
const uint16_t long_press_debounce_ms = 200; //ms

void clear_buttons_event(uint8_t button_number) {
	buttons_state[button_number].long_press =0;
	buttons_state[button_number].medium_press =0;
	buttons_state[button_number].short_press =0;


}

void read_buttons_state() {
	buttons_state[ENCODER].actual_state = !HAL_GPIO_ReadPin(ENC_PB_GPIO_Port,
			ENC_PB_Pin);
	buttons_state[JOY1].actual_state = !HAL_GPIO_ReadPin(JOY1_PB_GPIO_Port,
			JOY1_PB_Pin);
	buttons_state[JOY2].actual_state = !HAL_GPIO_ReadPin(JOY2_PB_GPIO_Port,
			JOY2_PB_Pin);

}

void process_buttons() {

	read_buttons_state();

	for (int i = 0; i < NUM_OF_BUTTONS; i++) {

		// decrement debounce of long press to not count short

		if (buttons_state[i].long_press_debounce)
			buttons_state[i].long_press_debounce -= 5;

		if (buttons_state[i].last_state) {
			buttons_state[i].tick_ms += 5;

			if (!(buttons_state[i].actual_state)
					|| (buttons_state[i].tick_ms > LONG_PRESS)) {

				if (buttons_state[i].tick_ms > SHORT_PRESS
						&& buttons_state[i].tick_ms < MEDIUM_PRESS
						&& (!buttons_state[i].long_press_debounce)) {

					//SHORT PRESS ACTION
					buttons_state[i].short_press = 1;
					printf("Short press button %d\r\n", i);

				} else if (buttons_state[i].tick_ms > MEDIUM_PRESS
						&& buttons_state[i].tick_ms < LONG_PRESS) {

					//MEDIUM PRESS ACTION
					buttons_state[i].medium_press = 1;
					printf("Medium press %d \r\n", i);
				}

				else if (buttons_state[i].tick_ms > LONG_PRESS) {

					//LONG PRESS ACTION
					buttons_state[i].long_press_debounce = long_press_debounce_ms;
					buttons_state[i].long_press = 1;

					printf("Long press  %d\r\n", i);

				}

				buttons_state[i].tick_ms = 0;
			}
		}
		buttons_state[i].last_state = buttons_state[i].actual_state;
	}
}
