/*
 * sbus.h
 *
 *  Created on: 02.11.2020
 *      Author: Damian
 */

#ifndef SBUS_H_
#define SBUS_H_

#include "main.h"

#define SBUS_HEADER 0x0F
#define SBUS_FOOTER 0x00

void parse_sbus_data(uint16_t* channels, uint8_t *packet);

#endif /* SBUS_H_ */
