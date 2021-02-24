/*
 * binding.c
 *
 *  Created on: 17.02.2021
 *      Author: Damiano
 */

#include "binding.h"
#include "rc_controler.h"

#include <string.h>

extern UART_HandleTypeDef huart3;

#define SYNC_VALUE 0xCD

uint8_t binding_key_reciver[8] =
		{ 10, 20, 70, 90, 220, 71, 77, 81 };
uint8_t binding_key_controler[8] =
		{ 99, 102, 1, 21, 34, 90, 111, 110 };

void bind_event_controler() {

	static uint8_t is_bind_key_correct = 0;
	uint8_t retries = 100;
	uint8_t sync_frame_tx = SYNC_VALUE;
	uint8_t sync_frame_rx = 0;

	//Sync frame
	while (sync_frame_rx != SYNC_VALUE)
	{
		HAL_UART_Transmit(&huart3, &sync_frame_tx, 1, 100);
		HAL_Delay(50);
		HAL_UART_Receive(&huart3, &sync_frame_rx, 1, 100);

	}

//	// Wait for sending binding data until reciving proper key
//
//	while (retries && !is_bind_key_correct)
//	{
//		retries--;
//		//Send handshake key
//		HAL_UART_Transmit(&huart3, binding_key_controler, SIZE(binding_key_controler), 100);
//		HAL_Delay(10);
//		//Reive confirmation key
//		HAL_UART_Receive(&huart3, rc_status.bindnig_data_rx, SIZE(rc_status.bindnig_data_rx), 100);
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
	//1. Sending  tx-pipe adress

	HAL_UART_Transmit(&huart3, tx_pipe_adress.frame, SIZE(tx_pipe_adress.frame), 100);
	HAL_Delay(1);

	//2. Sending  rx-pipe adress

	//3. Sending channel number
	//			HAL_UART_Transmit(&huart2, rx_pipe_adress.frame, SIZE(rx_pipe_adress.frame),100);
	//			HAL_Delay(1);

}

//void bind_event_reciver() {
//
//
//	// Try as long as still have retries or recived response
//	while (retries && !is_response)
//	{
//
//		retries--;
//
//		// Check if we  received frame
//
//		if (HAL_UART_Receive(&huart1, reciver_status.bindnig_data_rx,
//				sizeof(reciver_status.bindnig_data_rx), 100) != HAL_ERROR)
//		{
//			// Check if it looks just as should ( security purposes) // //TO DO -> add define?? and compare funciton?
//			if (reciver_status.bindnig_data_rx[0] == 1
//					&& reciver_status.bindnig_data_rx[3] == 7)
//				is_response = 1;
//
//		}
//
//	}
//	// Check if frame was correct response with some data
//	if (is_response)
//	{
//
//		memcpy(reciver_status.bindnig_data_tx, binding_data_tx, sizeof(binding_data_tx));
//
//		HAL_UART_Transmit(&huart1, reciver_status.bindnig_data_tx,
//				sizeof(reciver_status.bindnig_data_tx), 100);
//
//	}
//	HAL_Delay(10);
//	// Get the pipe adress and other settings (TO DO other settings)
//	if (HAL_UART_Receive(&huart1, reciver_status.bindnig_data_rx,
//			sizeof(reciver_status.bindnig_data_rx), 100) != HAL_ERROR)
//	{
//		memcpy(&txPipeAdress, reciver_status.bindnig_data_rx, sizeof(txPipeAdress));
//
//	}
//
//}
