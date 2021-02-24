/*
 * rc_reciver.h
 *
 *  Created on: 08.02.2021
 *      Author: Damiano
 *
 * File contains main rc_reciver functions, logic and sturctures
 *
 *
 */

#ifndef RC_RECIVER_H_
#define RC_RECIVER_H_

#include "main.h"
#include "nrf.h"

extern uint8_t binding_data_tx[8];
//RECIVER -> RX
extern uint64_t rxPipeAdress;
extern uint64_t txPipeAdress;

//PIPES FOR NRF24
typedef union _PipeAdress {
	uint8_t frame[8];
	uint64_t var;

} PipeAdress;

typedef enum {
	CONTROLER, RECIVER
} rc_role;
typedef struct _RadioConfig {

	PipeAdress tx_pipe;
	PipeAdress rx_pipe;

	rf24_crclength_e crc_length;
	rf24_datarate_e datarate;
	rf24_pa_dbm_e pa_dbm;

	uint8_t channel;
	uint8_t auto_ack;
	rc_role role;

} RadioConfig;

typedef enum {
	NORMAL_MODE, BIND_MODE, DEBUG_MODE
} rc_reciver_mode;

typedef struct ReciverStatus {
	uint16_t raw_rx_data[16]; //32 byte payload from nrf24
	uint8_t sbus_transmition_frame[25];
	uint8_t signal_lost;
	uint8_t frame_lost;

	uint32_t frames_recived;
	rc_reciver_mode mode;

	uint8_t bindnig_data_rx[8];
	uint8_t bindnig_data_tx[8];

} ReciverStatus;

extern ReciverStatus reciver_status;

extern PipeAdress rx_pipe_adress;
extern PipeAdress tx_pipe_adress;

void set_reciver_binding_mode();
void set_reciver_normal_mode();

void confirm_binding_mode();

#endif /* RC_RECIVER_H_ */
