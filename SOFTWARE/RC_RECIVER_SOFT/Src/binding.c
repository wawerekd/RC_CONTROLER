/*
 * binding.c
 *
 *  Created on: 17.02.2021
 *      Author: Damiano
 */

#include "binding.h"
#include "rc_reciver.h"

#include <string.h>

uint8_t binding_key_reciver[8] =
		{ 10, 20, 70, 90, 220, 71, 77, 81 };
uint8_t binding_key_controler[8] =
		{ 99, 102, 1, 21, 34, 90, 111, 110 };

extern UART_HandleTypeDef huart1;


#define SYNC_VALUE 0xCD
//void bind_event_controler() {
//
//	static uint8_t is_bind_key_correct = 0;
//	static uint8_t retries = 100;
//
//	// Wait for sending binding data until reciving proper key
//
//	while (retries && !is_bind_key_correct)
//	{
//		retries--;
//		//Send handshake key
//		HAL_UART_Transmit(&huart2, binding_key_controler, SIZE(binding_key_controler), 100);
//		HAL_Delay(10);
//		//Reive confirmation key
//		HAL_UART_Receive(&huart2, rc_status.bindnig_data_rx, SIZE(rc_status.bindnig_data_rx), 100);
//
//		//Compare keys
//		if (!memcmp(rc_status.bindnig_data_rx, binding_key_reciver,
//				sizeof(rc_status.bindnig_data_rx)))
//			is_bind_key_correct = 1;
//	}
//
//	//Sending data for binding if recived correct key
//	if (is_bind_key_correct)
//	{
//		//1. Sending  tx-pipe adress
//
//		HAL_UART_Transmit(&huart2, tx_pipe_adress.frame, SIZE(rx_pipe_adress.frame), 100);
//		HAL_Delay(1);
//
//		//2. Sending  rx-pipe adress
//
//		//3. Sending channel number
//		//			HAL_UART_Transmit(&huart2, rx_pipe_adress.frame, SIZE(rx_pipe_adress.frame),100);
//		//			HAL_Delay(1);
//
//	}
//
//}

void bind_event_reciver() {


	uint8_t sync_frame_rx = 0;


     while(sync_frame_rx!=SYNC_VALUE){

    	 HAL_UART_Receive(&huart1, &sync_frame_rx, 1, 10);
     }
     //Sync delay
     HAL_Delay(50);
     //Send back what recived to confirm
     HAL_UART_Transmit(&huart1, &sync_frame_rx, 1, 10);



	if (HAL_UART_Receive(&huart1, reciver_status.bindnig_data_rx,
			SIZE(reciver_status.bindnig_data_rx), 100) != HAL_ERROR)
	{
		memcpy(&txPipeAdress, reciver_status.bindnig_data_rx, sizeof(txPipeAdress));

	}



}
