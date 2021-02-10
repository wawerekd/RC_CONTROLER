/*
 * sbus.h
 *
 *  Created on: 02.11.2020
 *      Author: Damian
 */

#ifndef SBUS_H_
#define SBUS_H_

#include "main.h"
#include "common.h"

//CHANNEL VALUES
#define RC_CHANNEL_MIN 1000
#define RC_CHANNEL_MAX 2000


//SBUS DEFINES
#define SBUS_MIN_OFFSET 173
#define SBUS_MID_OFFSET 992
#define SBUS_MAX_OFFSET 1811


#define SBUS_CHANNEL_NUMBER 16
#define SBUS_PACKET_LENGTH 25


#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00
#define SBUS_FRAME_FOOTER_V2 0x04
#define SBUS_STATE_FAILSAFE 0x08
#define SBUS_STATE_SIGNALLOSS 0x04



void parse_sbus_data(uint16_t* channels, uint8_t *packet);

#endif /* SBUS_H_ */
