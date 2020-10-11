/*
 * oled.h
 *
 *
 *
 *  Created on: 19.01.2020
 *      Author: Damian
 * Sourece files to manage usage of OLED Screen SSD1306
 *
 *
 */

#ifndef OLED_H_
#define OLED_H_

#include "main.h"  //PIN defines and HAL_Libaries
#include <stdio.h>

#include "ssd1306.h"   // driver for OLED
#include "fonts.h"     // fonts declaration
#include "test.h"      // for testing

//Private functions

void oledInit(void);
void oledPrintInitScreen(void);
void oledDrawValueBars(uint16_t value1,uint16_t value2,uint16_t value3,uint16_t value4);

//Initial Screen




#endif /* OLED_H_ */
