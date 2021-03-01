/*
 * binding.c
 *
 *  Created on: 17.02.2021
 *      Author: Damiano
 */

#include "binding.h"
//#include "rc_controler.h"

#include <string.h>

extern UART_HandleTypeDef huart3;

#define SYNC_VALUE 0xCD



static void send_nrf24_configuration(NRF24_InitStruct * const reciver_config, UART_HandleTypeDef *bind_huart)
{

	//1. Sending  tx-pipe adress

		HAL_UART_Transmit(bind_huart, reciver_config->tx_pipe.frame, SIZE( reciver_config->tx_pipe.frame), 100);
		HAL_Delay(10);

		//2. Sending  rx-pipe adress
		HAL_UART_Transmit(bind_huart, reciver_config->rx_pipe.frame, SIZE( reciver_config->rx_pipe.frame), 100);
		HAL_Delay(10);

		//3. Sending channel number

		HAL_UART_Transmit(bind_huart, &reciver_config->radio_channel, SIZE( reciver_config->rx_pipe.frame), 100);
				HAL_Delay(10);

}

void bind_event_controler(UART_HandleTypeDef * bind_huart,NRF24_InitStruct * const reciver_config) {


	// HW init to add? -

	uint8_t sync_frame_tx = SYNC_VALUE;
	uint8_t sync_frame_rx = 0;

	//Sync frame
	while (sync_frame_rx != SYNC_VALUE)
	{
		HAL_UART_Transmit(bind_huart, &sync_frame_tx, 1, 100);
		HAL_Delay(50);
		HAL_UART_Receive(bind_huart, &sync_frame_rx, 1, 100);

	}
	send_nrf24_configuration(reciver_config,bind_huart);


}

