/*
 * rc_reciver.c
 *
 *  Created on: 08.02.2021
 *      Author: Damiano
 */
#include "rc_reciver.h"

uint8_t binding_data_tx[8] =
		{ 100, 102, 103, 104, 55, 66, 77, 88 };

uint64_t rxPipeAdress = 0xABCDABCD72LL;
uint64_t txPipeAdress = 0x544d52687CLL;

PipeAdress rx_pipe_adress =
{ .var = 0xABCDABCD72LL };

PipeAdress tx_pipe_adres =
{ .var = 0x544d52687CLL };

void set_reciver_normal_mode() {

	reciver_status.mode = NORMAL_MODE;
	//Start with no singal, if present we can change
	reciver_status.signal_lost = 1;

}

void set_reciver_binding_mode() {

	reciver_status.mode = BIND_MODE;

}

void confirm_binding_mode() {

	HAL_GPIO_WritePin(LD_GREEN_GPIO_Port, LD_GREEN_Pin, SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(LD_GREEN_GPIO_Port, LD_GREEN_Pin, RESET);
	HAL_Delay(200);
	HAL_GPIO_WritePin(LD_GREEN_GPIO_Port, LD_GREEN_Pin, SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(LD_GREEN_GPIO_Port, LD_GREEN_Pin, RESET);

}
