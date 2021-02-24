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

#include "rc_controler.h"

//Private functions

void oledInit(void);
void oledPrintInitScreen(void);
void oledDrawValueBars(uint16_t value1, uint16_t value2, uint16_t value3,
		uint16_t value4, uint8_t start_number);
void oledPrintEncValues(uint8_t rotation_value, uint8_t push_value);
void oledPrintMainScreen(RC_Controler_Status * rc_status);
void oledPrintCalibScreen(uint8_t channel_number, uint16_t actual_value ,uint16_t time_to_end,
		uint16_t min, uint16_t max);
void oledPrintCalibMenu(uint8_t active_channel_number, uint8_t row);

void oledPrintBindScren();

//Initial Screen

#endif /* OLED_H_ */
