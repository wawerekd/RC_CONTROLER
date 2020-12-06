/*
 * eeporm_store.h
 *
 *  Created on: 22.10.2020
 *      Author: Damian
 */

#ifndef EEPORM_STORE_H_
#define EEPORM_STORE_H_

#include "main.h"
#include "rc_controler.h"




extern I2C_HandleTypeDef hi2c1;
extern RC_Channels rc_channels;



//DEVICE ADRESS




uint8_t read_data_from_epprom();
uint8_t wrtie_data_to_epprom();
void read_initial_store();
void update_eeporm_store();
void clear_store();


#endif /* EEPORM_STORE_H_ */
