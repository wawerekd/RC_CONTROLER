/*
 * eeprom_store.c
 *
 *  Created on: 22.10.2020
 *      Author: Damian
 */
#include "eeporm_store.h"

const uint16_t EEPROM_ADDRESS = 0xA0;

Calib_Data calibration_values;

void read_initial_store() {
	//EEPROM  - TO DO as lib

	//readinga  channel calibration calibration values
	HAL_I2C_Mem_Read(&hi2c1, EEPROM_ADDRESS, 0, 0xFF,
			calibration_values.calib_raw_data, 44, 10);

	for (int i = 0; i < 11; i++) {
		rc_channels.calibration_values[i] =
				calibration_values.calibration_values_min_max[i];
	}
}

void update_eeporm_store() {
	for (int i = 0; i < 11; i++) {
		calibration_values.calibration_values_min_max[i] =
					rc_channels.calibration_values[i];
		}

	HAL_I2C_Mem_Write(&hi2c1, EEPROM_ADDRESS, 0, 0xFF,
			calibration_values.calib_raw_data, 44, 10);

}
void clear_store();

