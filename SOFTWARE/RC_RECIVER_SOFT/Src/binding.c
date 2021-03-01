/*
 * binding.c
 *
 *  Created on: 17.02.2021
 *      Author: Damiano
 */

#include "binding.h"
#include "rc_reciver.h"

#include <string.h>

//uint8_t binding_key_reciver[8] =
//		{ 10, 20, 70, 90, 220, 71, 77, 81 };
//uint8_t binding_key_controler[8] =
//		{ 99, 102, 1, 21, 34, 90, 111, 110 };

extern UART_HandleTypeDef huart1;


#define SYNC_VALUE 0xCD


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
