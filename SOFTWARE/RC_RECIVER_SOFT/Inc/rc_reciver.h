/*
 * rc_reciver.h
 *
 *  Created on: 08.02.2021
 *      Author: Damiano
 *
 * File contains main rc_reciver functions, logic and sturctures
 *
 *
 */

#ifndef RC_RECIVER_H_
#define RC_RECIVER_H_

#include "main.h"

enum rc_reciver_mode {NORMAL_MODE, BIND_MODE, DEBUG_MODE };

typedef struct ReciverStatus {
	uint16_t raw_rx_data[16]; //32 byte payload from nrf24
	uint8_t sbus_transmition_frame[25];
	uint8_t signal_lost;
	uint8_t frame_lost;
	uint32_t frames_recived;
} ReciverStatus;



#endif /* RC_RECIVER_H_ */
