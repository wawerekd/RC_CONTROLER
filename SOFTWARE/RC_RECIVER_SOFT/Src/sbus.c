/*
 * sbus.c
 *
 *  Created on: 02.11.2020
 *      Author: Damian
 */

#include "sbus.h"

//uint16_t map_values(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
//
//	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
//
//}


void parse_sbus_data(uint16_t* channels, uint8_t *packet)
{
	/*
	 * Map 1000-2000 with middle at 1500 chanel values to
	 * 173-1811 with middle at 992 S.BUS protocol requires
	 */

	static uint16_t output[SBUS_CHANNEL_NUMBER] =
	{ 0 };

	for (uint8_t i = 0; i < SBUS_CHANNEL_NUMBER; i++)
	{
		output[i] = map_values(channels[i], RC_CHANNEL_MIN, RC_CHANNEL_MAX, SBUS_MIN_OFFSET,
				SBUS_MAX_OFFSET);
	}

	/* assemble the SBUS packet */
	// SBUS header
	packet[0] = SBUS_HEADER;
	// 16 channels of 11 bit data
	if (channels)
	{
		packet[1] = (uint8_t) (output[0] & 0x07FF);
		packet[2] = (uint8_t) ((output[0] & 0x07FF) >> 8 | (output[1] & 0x07FF) << 3);
		packet[3] = (uint8_t) ((output[1] & 0x07FF) >> 5 | (output[2] & 0x07FF) << 6);
		packet[4] = (uint8_t) ((output[2] & 0x07FF) >> 2);
		packet[5] = (uint8_t) ((output[2] & 0x07FF) >> 10 | (output[3] & 0x07FF) << 1);
		packet[6] = (uint8_t) ((output[3] & 0x07FF) >> 7 | (output[4] & 0x07FF) << 4);
		packet[7] = (uint8_t) ((output[4] & 0x07FF) >> 4 | (output[5] & 0x07FF) << 7);
		packet[8] = (uint8_t) ((output[5] & 0x07FF) >> 1);
		packet[9] = (uint8_t) ((output[5] & 0x07FF) >> 9 | (output[6] & 0x07FF) << 2);
		packet[10] = (uint8_t) ((output[6] & 0x07FF) >> 6 | (output[7] & 0x07FF) << 5);
		packet[11] = (uint8_t) ((output[7] & 0x07FF) >> 3);
		packet[12] = (uint8_t) ((output[8] & 0x07FF));
		packet[13] = (uint8_t) ((output[8] & 0x07FF) >> 8 | (output[9] & 0x07FF) << 3);
		packet[14] = (uint8_t) ((output[9] & 0x07FF) >> 5 | (output[10] & 0x07FF) << 6);
		packet[15] = (uint8_t) ((output[10] & 0x07FF) >> 2);
		packet[16] = (uint8_t) ((output[10] & 0x07FF) >> 10 | (output[11] & 0x07FF) << 1);
		packet[17] = (uint8_t) ((output[11] & 0x07FF) >> 7 | (output[12] & 0x07FF) << 4);
		packet[18] = (uint8_t) ((output[12] & 0x07FF) >> 4 | (output[13] & 0x07FF) << 7);
		packet[19] = (uint8_t) ((output[13] & 0x07FF) >> 1);
		packet[20] = (uint8_t) ((output[13] & 0x07FF) >> 9 | (output[14] & 0x07FF) << 2);
		packet[21] = (uint8_t) ((output[14] & 0x07FF) >> 6 | (output[15] & 0x07FF) << 5);
		packet[22] = (uint8_t) ((output[15] & 0x07FF) >> 3);


	}
	//TO DO
	// flags  - to update failsafe or sth
	packet[23] = 0x00;
	// footer
	packet[24] = SBUS_FOOTER;

}
