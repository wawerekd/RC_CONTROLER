/*
 * buttonProcess.h
 *
 *  Created on: 24.01.2020
 *      Author: dwawerek
 */
#include "main.h"

#ifndef BUTTONPROCESS_H_
#define BUTTONPROCESS_H_



#define SHORT_PRESS   5   //*10ms
#define MEDIUM_PRESS  80   //*10ms
#define LONG_PRESS    200  //*10ms



#define NUM_OF_BUTTONS 2

enum BUTTONS {
	BUTTON1, JOY1_PB
};






void buttons_state_update(uint8_t* buttons_state);
void process_buttons();













#endif /* BUTTONPROCESS_H_ */
