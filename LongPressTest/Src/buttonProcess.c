/*
 * buttonProcess.c
 *
 *  Created on: 24.01.2020
 *      Author: dwawerek
 */

#include "buttonProcess.h"

extern UART_HandleTypeDef huart2;
extern char uart_buff[50];
extern uint8_t buttons_states[NUM_OF_BUTTONS];

void process_buttons()
{

	static uint16_t tick_counters[NUM_OF_BUTTONS] =
	{ 0 };
	static uint8_t longPressDebounce[NUM_OF_BUTTONS] =
	{ 0 };

	static volatile uint8_t buttons_last_state[NUM_OF_BUTTONS] =
	{ 1, 1 };

	//tick_counter++;
	for (int i = 0; i < NUM_OF_BUTTONS; i++)
	{
		if (longPressDebounce[i])
			longPressDebounce[i]--;

		if (buttons_last_state[0])
		{
			tick_counters[i]++;

			if (!buttons_states[i] || (tick_counters[i] > LONG_PRESS))
			{

				if (tick_counters[i] > SHORT_PRESS && tick_counters[i] < MEDIUM_PRESS && (!longPressDebounce[i]))
				{

					//SHORT PRESS ACTION
					sprintf(uart_buff, "Short press \n");

					HAL_UART_Transmit(&huart2, (uint8_t*) uart_buff, strlen(uart_buff),
					HAL_MAX_DELAY);

				}
				else if (tick_counters[i] > MEDIUM_PRESS && tick_counters[i] < LONG_PRESS)
				{

					//MEDIUM PRESS ACTION
					sprintf(uart_buff, "Medium press \n");

										HAL_UART_Transmit(&huart2, (uint8_t*) uart_buff, strlen(uart_buff),
										HAL_MAX_DELAY);
				}

				else if (tick_counters[i] > LONG_PRESS)
				{

					//LONG PRESS ACTION
					longPressDebounce[i] = 70;

					sprintf(uart_buff, "Long press \n");

					HAL_UART_Transmit(&huart2, (uint8_t*) uart_buff, strlen(uart_buff),
					HAL_MAX_DELAY);
				}

				tick_counters[i] = 0;
			}
		}
		buttons_last_state[i] = buttons_states[0];
	}
}
