#include "rc_controler.h"

RC_Channels rc_channels;

uint16_t map_values(uint16_t x, uint16_t in_min, uint16_t in_max,
		uint16_t out_min, uint16_t out_max) {

	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;

}
uint16_t map_values_sbus(uint16_t x, uint16_t in_min, uint16_t in_max) {
	uint16_t temp_value = (x - in_min) * (2000 - 1000) / (in_max - in_min) + 1000;
	if(temp_value> 2000)
		temp_value=2000;
	if(temp_value<1000)
		temp_value=1000;

	return temp_value;

}
uint16_t reverse_channel(uint16_t value) {
	return 3000-value;
}

void update_rc_mode(RC_Mode mode) {

	if (mode) { // checking if mode >0
		rc_channels.rc_mode = mode;
	}
}

void update_rc_channels(uint16_t* adc_values) {

	if (rc_channels.rc_mode == RC_SIMPLE_JOYSTICK) {
		rc_channels.scaled_values[0] = reverse_channel(map_values_sbus(adc_values[6], 0, 3200)); // ROLL
		rc_channels.scaled_values[1] = (map_values_sbus(adc_values[5], 0, 3200)); // PITCH
		rc_channels.scaled_values[2] = reverse_channel(map_values_sbus(adc_values[7], 0, 3200)); // THROTLE
		rc_channels.scaled_values[3] = map_values_sbus(adc_values[8], 0, 3200); // YAW

	} else if (rc_channels.rc_mode == RC_SIMPLE_JOYSTICK) {
		rc_channels.scaled_values[0] = map_values_sbus(adc_values[7], 0, 3200); // ROLL
		rc_channels.scaled_values[1] = map_values_sbus(adc_values[6], 0, 3200); // PITCH
		rc_channels.scaled_values[2] = map_values_sbus(adc_values[5], 0, 3200); // THROTLE
		rc_channels.scaled_values[3] = map_values_sbus(adc_values[8], 0, 3200); // YAW

	}

	rc_channels.scaled_values[4] = map_values_sbus(adc_values[0], 0, 3646); // POT1 //VALUES READ FROM MANUAL CALIB
	rc_channels.scaled_values[5] = map_values_sbus(adc_values[1], 0, 3660); // POT2 //VALUES READ FROM MANUAL CALIB

	rc_channels.scaled_values[6] = map_values_sbus(adc_values[2], 0, 4000); // SW1
	rc_channels.scaled_values[7] = map_values_sbus(adc_values[3], 0, 4000); // SW2

	rc_channels.scaled_values[8] = map_values_sbus(adc_values[9], 0, 4000); // 	 SW3
	rc_channels.scaled_values[10] = map_values_sbus(adc_values[10], 0, 4000); // SW4
	// BAT_LEVEL - DO NOT USE AS INPUT

    //	rc_channels.scaled_values[8] = map_values_sbus(adc_values[4], 0, 3200);

	for(int i=0 ;i <4 ; i++){
		rc_channels.low_pass_values[i]= rc_channels.low_pass_values[i]*0.95 + 0.05*rc_channels.scaled_values[i];
	}








}
;

