/*
 * CAN.h
 *
 *  Created on: 24/03/2019
 *      Author: Cognati
 */

#include "S32K144.h" /* include peripheral declarations S32K144 */
#include "CAN.h"/** Include the library that includes the API to use the CAN protocol*/
#include "LPSPI.h"
#include "clocks_and_modes.h"

 /**It is necessary  to select which device is going to be used  */
#define DEVICE_1 1
#define DEVICE_2 0


/**This program is able to test the CAN protocol making transmissions and receptions with other devices*/
int main(void) {

	uint32_t rx_msg_count = 0;
	CAN_transmission_config_t trans_configuration;
	static CAN_reception_t reception_data;

	WDOG_disable();
	SOSC_init_8MHz(); /* Initialize system oscillator for 8 MHz xtal */
	SPLL_init_160MHz(); /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	NormalRUNmode_80MHz(); /* Init clocks: 80 MHz sysclk & core, 40 MHz bus, 20 MHz flash */
	init_CAN(0, 0x555, 250000); /* Initialize CAN peripheral with initial conditions, which they are the frequency and the ID_RX*/
	PORT_init_CAN0(); /* Configure ports */
	LPSPI1_init_master(); /* Initialize LPSPI1 for communication with MC33903 */
	LPSPI1_init_MC33903(); /* Configure SBC via SPI for CAN transceiver operation */

	trans_configuration.can_selected=CAN0;
	trans_configuration.DLC=8;
	trans_configuration.id_standar=0x111;
	trans_configuration.word1=0xB;
	trans_configuration.word2=0xB;


	reception_data.can_pointer=CAN0;

	setID_Rx(0x524, 2);
	setID_Rx(0x234, 3);


#if DEVICE_1 /** Verify if this program is going to be used for the DEVICE 1*/
	transmit_CAN(&trans_configuration); /** The DEVICE 1  makes a transmission */
#endif
	for (;;) { /* Loop: if a msg is received, transmit a msg */
		if ((CAN0->IFLAG1 >> 4) & 1) { /* If CAN 0 MB 4 flag is set (received msg), read MB4 */
#if DEVICE_1
			receive_CAN(&reception_data); /** The DEVICE 1  makes a reception */
#elif DEVICE_2 /** Verify if this program is going to be used for the DEVICE 2*/
			receive_CAN(&reception_data); /** The DEVICE 2  makes a reception */
#endif
			rx_msg_count++; /* Increment receive msg counter */
			if (rx_msg_count == 1000) { /* If 1000 messages have been received, */
				PTD->PTOR |= 1 << 16; /*   toggle output port D16 (Green LED) */
				rx_msg_count = 0; /*   and reset message counter */
			}
#if DEVICE_1
			transmit_CAN(&trans_configuration);  /** The DEVICE 1  makes a transmission */
#elif DEVICE_2
			transmit_CAN(&trans_configuration);  /** The DEVICE 2  makes a transmission */
#endif
		}
	}
}
