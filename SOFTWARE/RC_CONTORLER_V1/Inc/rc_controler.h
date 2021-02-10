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
typedef enum {
	RC_SIMPLE_JOYSTICK = 1, RC_DIV_JOYSTICK, RC_IMU
} RC_Mode;

typedef struct Channel_Calib {
	uint16_t min;
	uint16_t max;
} Channel_Calib;

typedef union Calib_Data {
	Channel_Calib calibration_values_min_max[11];
	uint8_t calib_raw_data[44];
} Calib_Data;

typedef struct RC_Channels {
	uint16_t scaled_values[11];
	uint16_t low_pass_values[11];
	Channel_Calib calibration_values[11];
	RC_Mode rc_mode;

} RC_Channels;

typedef struct RC_Controler_Status {
	uint32_t frames_sent;
	uint32_t frames_lost;
	uint32_t acks_recived;
	uint8_t mpu_init_succes;
	uint8_t rc_recvier_found;

} RC_Controler_Status;

uint16_t map_values(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
uint16_t map_values_sbus(uint16_t x, uint16_t in_min, uint16_t in_max);
// make a separte file for RC_channels
uint16_t reverse_channel(uint16_t value);
void update_rc_channels(uint16_t* adc_values);
void update_rc_mode(RC_Mode mode);

void calibrate_channel(uint8_t channel_number, uint16_t timeout);


#endif /* RC_CONTROLER_H_ */
