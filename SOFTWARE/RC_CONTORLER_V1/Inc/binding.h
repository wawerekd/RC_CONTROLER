/*
 * binding.h
 *
 *  Created on: 17.02.2021
 *      Author: Damiano
 */

/*
 * Libary for binding procedure of RC_SYSTEM
 *
 * General instructions:
 *
 * Binding comes through one dedicated RC_Controler UART port.
 * It is needed to put RC_Controler in BINDING mode by choosing proper menu through configuration / long presses combination ( to be tested)
 * Being in binding mode should be shown by proper screen or blinking leds combination.
 *
 * Binded models ID and transmission pipe should be stored in EPPROM of RC_CONTROLER or in Flash of RC_RECIVER
 *
 * 1. To put RC_RECIVER in BINDING mode it should be connected to deditaced port in RC_Controler with BIND button pressed.
 * 2. After being connected RC_RECIVER sends handshake frame with current ID and RC_CONTROLER sends back confirmation
 * 3. Then RC_CONTROLER send ID and TxPipe number
 * 4. Then RC_Reciver confirms ID by sending it back
 * 5. After confirmation data is stored in FLASH or EPPROM and devices go out of Binding mode and passes special sequance of events(Screen/led blinks) to show end of conversion
 *
 *
*/


#ifndef BINDING_H_
#define BINDING_H_

typedef enum SystemRole { RC_RECIVER, RC_CONTROLER};

typedef struct _RC_System_Member{

	SystemRole system_role;
	uint32_t system_ID;
	uint64_t nrf_pipe_adress;

}_RC_System_Member;







#endif /* BINDING_H_ */
