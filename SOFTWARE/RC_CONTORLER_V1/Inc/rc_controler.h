/*
 * rc_controler.h
 *
 * Liblary for main functionalities of RC_Controler
 *  Created on: 07.10.2020
 *      Author: Damian
 */

#ifndef RC_CONTROLER_H_
#define RC_CONTROLER_H_

#include "main.h"
typedef enum   {RC_SIMPLE_JOYSTICK = 1, RC_DIV_JOYSTICK, RC_IMU} RC_Mode;

typedef struct RC_Channels {
	uint16_t scaled_values[11];
	uint16_t low_pass_values[11];
	RC_Mode rc_mode;

} RC_Channels;

uint16_t map_values(uint16_t x, uint16_t in_min, uint16_t in_max,
		uint16_t out_min, uint16_t out_max);
uint16_t map_values_sbus(uint16_t x, uint16_t in_min, uint16_t in_max);




uint16_t reverse_channel(uint16_t value);
void update_rc_channels(uint16_t* adc_values);
void update_rc_mode(RC_Mode mode);

#endif /* RC_CONTROLER_H_ */
