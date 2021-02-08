/*
 * common.c
 *
 *  Created on: 08.02.2021
 *      Author: Damiano
 */

//Libary for most common opertions and functions know for example from Arduino etc.


#include "common.h"

uint16_t map_values(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}
