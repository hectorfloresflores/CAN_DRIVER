/*
 * CAN.c
 *
 *  Created on: 24/03/2019
 *      Author: Cognati
 */

#include "CAN.h"
#include "S32K144.h" /* include peripheral declarations S32K144 */


uint32_t  RxTIMESTAMP;


#define MSG_BUF_SIZE  4    /* Msg Buffer Size. (CAN 2.0AB: 2 hdr +  2 data= 4 words) */
#define MB_CAN_BUFF_CONFIG 0xc480000 /* EDL,BRS,ESI=0: CANFD not used */
									 /* CODE=0xC: Activate msg buf to transmit */
									 /* IDE=0: Standard ID */
									 /* SRR=1 Tx frame (not req'd for std ID) */
									 /* RTR = 0: data, not remote tx request frame*/


#define ID_MSG_SHIFT 18
#define _32SHIFT 32
#define _16SHIFT 16
#define _32_BITS_MASK 0xFFFFFFFF
#define _32HIGH_BITS_OF_64_MASK 0xFFFFFFFF0000000
#define MB_TX 0
#define CODE_SHIFT_MASK 0x07000000
#define _24_SHIFT 24
#define TIMESTAMP_MASK 0x000FFFF
#define FLAG1_MB4_MASK 0x00000010
#define MB_RX 4
/*Can init defines*/
#define CTRL1_CAN 0x00DB0006


#define NODE_A        /* If using 2 boards as 2 nodes, NODE A & B use different CAN IDs */


void PORT_init_CAN0(void) {

	PCC->PCCn[PCC_PORTE_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clock for PORTE */
	PORTE->PCR[4] |= PORT_PCR_MUX(5); /* Port E4: MUX = ALT5, CAN0_RX */
	PORTE->PCR[5] |= PORT_PCR_MUX(5); /* Port E5: MUX = ALT5, CAN0_TX */
	PCC->PCCn[PCC_PORTD_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clock for PORTD */
	PORTD->PCR[16] = 0x00000100; /* Port D16: MUX = GPIO (to green LED) */
	PTD->PDDR |= 1 << 16; /* Port D16: Data direction = output */
	PCC->PCCn[PCC_PORTB_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clock for PORTB */
	PORTB->PCR[14] |= PORT_PCR_MUX(3); /* Port B14: MUX = ALT3, LPSPI1_SCK */
	PORTB->PCR[15] |= PORT_PCR_MUX(3); /* Port B15: MUX = ALT3, LPSPI1_SIN */
	PORTB->PCR[16] |= PORT_PCR_MUX(3); /* Port B16: MUX = ALT3, LPSPI1_SOUT */
	PORTB->PCR[17] |= PORT_PCR_MUX(3); /* Port B17: MUX = ALT3, LPSPI1_PCS3 */
}



void init_CAN(uint8_t CAN_base, uint16_t id_Rx, uint64_t frecuency) {

	uint32_t   i=0;

	uint64_t presdiv = 8000000/(16*frecuency)-1;

	switch(CAN_base){

		case myCAN0:

			 /* CGC=1: enable clock to FlexCAN0 */
			 PCC->PCCn[PCC_FlexCAN0_INDEX] |= PCC_PCCn_CGC_MASK;
			 /* MDIS=1: Disable module before selecting clock */
			 CAN0->MCR |= CAN_MCR_MDIS_MASK;
			 /* CLKSRC=0: Clock Source = oscillator (8 MHz) */
			 CAN0->CTRL1 &= ~CAN_CTRL1_CLKSRC_MASK;
			 /* MDIS=0; Enable module config. (Sets FRZ, HALT)*/
			 CAN0->MCR &= ~CAN_MCR_MDIS_MASK;
			 /*Wait for FRZACK=1 on freeze mode entry/exit */
			 while (!((CAN0->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT))  {}
			 /* Configure for 500 KHz bit time */
			 CAN0->CTRL1 = CTRL1_CAN | (presdiv<<CAN_CTRL1_PRESDIV_SHIFT);
			 /*Clean Buffer*/
			 /* In FRZ mode, init CAN0 16 msg buf filters */
			 for( i=0; i<128; i++ ) {
			 /* Clear msg buf word */
			 CAN0->RAMn[i] = 0;
			 }
			 /* In FRZ mode, init CAN0 16 msg buf filters */
			 for( i=0; i<16; i++ ) {
			 CAN0->RXIMR[i] = 0xFFFFFFFF;
			 }
			 /* Global acceptance mask: check all ID bits */
			 CAN0->RXMGMASK = 0x1FFFFFFF;
			 /* Msg Buf 4, word 0: Enable for reception */
			 CAN0->RAMn[ 4*MSG_BUF_SIZE + 0] = 0x04000000;
			 CAN0->RAMn[ 4*MSG_BUF_SIZE + 1] = id_Rx<<ID_MSG_SHIFT;
			 /* PRIO = 0: CANFD not used */
			 /* Negate FlexCAN 1 halt state for 32 MBs */
			 CAN0->MCR = 0x0000001F;
			 /*Wait for FRZACK to clear (not in freeze mode) */
			 while ((CAN0->MCR && CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT)  {}
			 /*Wait for NOTRDY to clear (module ready)  */
			 while ((CAN0->MCR && CAN_MCR_NOTRDY_MASK) >> CAN_MCR_NOTRDY_SHIFT)  {}

			 break;

		case myCAN1:

			 /* CGC=1: enable clock to FlexCAN0 */
			 PCC->PCCn[PCC_FlexCAN1_INDEX] |= PCC_PCCn_CGC_MASK;
			 /* MDIS=1: Disable module before selecting clock */
			 CAN1->MCR |= CAN_MCR_MDIS_MASK;
			 /* CLKSRC=0: Clock Source = oscillator (8 MHz) */
			 CAN1->CTRL1 &= ~CAN_CTRL1_CLKSRC_MASK;
			 /* MDIS=0; Enable module config. (Sets FRZ, HALT)*/
			 CAN1->MCR &= ~CAN_MCR_MDIS_MASK;
			 /*Wait for FRZACK=1 on freeze mode entry/exit */
			 while (!((CAN1->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT))  {}
			 /* Configure for 500 KHz bit time */
			 CAN1->CTRL1 = CTRL1_CAN | (presdiv<<CAN_CTRL1_PRESDIV_SHIFT);
			 /*Clean Buffer*/
			 /* In FRZ mode, init CAN0 16 msg buf filters */
			 for( i=0; i<16; i++ ) {CAN1->RXIMR[i] = 0xFFFFFFFF;}
			 /* Global acceptance mask: check all ID bits */
			 CAN1->RXMGMASK = 0x1FFFFFFF;
			 /*Ckec Ids*/
			 /* In FRZ mode, init CAN0 16 msg buf filters */
			 for( i=0; i<16; i++ ) {	CAN1->RXIMR[i] = 0xFFFFFFFF;}
			 /* Global acceptance mask: check all ID bits */
			 CAN1->RXMGMASK = 0x1FFFFFFF;
			 /* Msg Buf 4, word 0: Enable for reception */
			 CAN1->RAMn[ 4*MSG_BUF_SIZE + 0] = 0x04000000;
			 CAN1->RAMn[ 4*MSG_BUF_SIZE + 1] = 0x14440000;
			 /* PRIO = 0: CANFD not used */
			 /* Negate FlexCAN 1 halt state for 32 MBs */
			 CAN1->MCR = 0x0000001F;
			 /*Wait for FRZACK to clear (not in freeze mode) */
			 while ((CAN1->MCR && CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT)  {}
			 /*Wait for NOTRDY to clear (module ready)  */
			 while ((CAN1->MCR && CAN_MCR_NOTRDY_MASK) >> CAN_MCR_NOTRDY_SHIFT)  {}

			 break;
		case myCAN2:

			 /* CGC=1: enable clock to FlexCAN0 */
			 PCC->PCCn[PCC_FlexCAN2_INDEX] |= PCC_PCCn_CGC_MASK;
			 /* MDIS=1: Disable module before selecting clock */
			 CAN2->MCR |= CAN_MCR_MDIS_MASK;
			 /* CLKSRC=0: Clock Source = oscillator (8 MHz) */
			 CAN2->CTRL1 &= ~CAN_CTRL1_CLKSRC_MASK;
			 /* MDIS=0; Enable module config. (Sets FRZ, HALT)*/
			 CAN2->MCR &= ~CAN_MCR_MDIS_MASK;
			 /*Wait for FRZACK=1 on freeze mode entry/exit */
			 while (!((CAN2->MCR & CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT))  {}
			 /* Configure for 500 KHz bit time */
			 CAN2->CTRL1 = CTRL1_CAN | (presdiv<<CAN_CTRL1_PRESDIV_SHIFT);
			 /*Clean Buffer*/
			 for(i=0; i<128; i++ ) {   /* CAN0: clear 32 msg bufs x 4 words/msg buf = 128 words*/
			     CAN0->RAMn[i] = 0;      /* Clear msg buf word */
			   }
			   for(i=0; i<16; i++ ) {          /* In FRZ mode, init CAN0 16 msg buf filters */
			     CAN0->RXIMR[i] = 0x0;  /* Check all ID bits for incoming messages */
			   }
			   CAN0->RXMGMASK = 0x0;  /* Global acceptance mask: check all ID bits */
			 /* Msg Buf 4, word 0: Enable for reception */
			 CAN2->RAMn[ 4*MSG_BUF_SIZE + 0] = 0x04000000;
			 CAN2->RAMn[ 4*MSG_BUF_SIZE + 1] = id_Rx<<ID_MSG_SHIFT;
			 /* PRIO = 0: CANFD not used */
			 /* Negate FlexCAN 1 halt state for 32 MBs */
			 CAN2->MCR = 0x0000001F;
			 /*Wait for FRZACK to clear (not in freeze mode) */
			 while ((CAN2->MCR && CAN_MCR_FRZACK_MASK) >> CAN_MCR_FRZACK_SHIFT)  {}
			 /*Wait for NOTRDY to clear (module ready)  */
			 while ((CAN2->MCR && CAN_MCR_NOTRDY_MASK) >> CAN_MCR_NOTRDY_SHIFT)  {}

			 break;
	}

}

void transmit_CAN(CAN_Type *can_selected, uint16_t id_standar, uint8_t DLC, uint32_t word1, uint32_t word2) {

    /**Only 8 bytes is permitted to the DLC parameter*/
	if(DLC>8)
	{
		DLC=8;
	}

	can_selected->IFLAG1 = CAN_IFLAG1_BUF0I_MASK; /**Clear the MB0 flag, which, is set when some transmission or reception has occurred*/
	can_selected->RAMn[MB_TX * MSG_BUF_SIZE + 3] = word2; /** Store the MB0 word2 into the embedded RAM memory */
	can_selected->RAMn[MB_TX * MSG_BUF_SIZE + 2] = word1; /** Store the MB0 word1 into the embedded RAM memory */
	can_selected->RAMn[MB_TX * MSG_BUF_SIZE + 1] = id_standar<<ID_MSG_SHIFT; /** Set the identifier of the message within the embedded RAM memory */
	uint32_t MB_configuration_aux =MB_CAN_BUFF_CONFIG & (~(0XF << _16SHIFT));/** Clean the DLC to set the new DLC value*/
	can_selected->RAMn[MB_TX * MSG_BUF_SIZE + 0] = MB_configuration_aux | (DLC<<_16SHIFT); /** Store the message buffer configuration and the new DLC value into the embedded RAM memory*/
	//can_selected->RAMn[MB_TX * MSG_BUF_SIZE + 0] = MB_CAN_BUFF_CONFIG;

}

void receive_CAN(CAN_Type *can_pointer,uint32_t*Rx_CODE,uint32_t*Rx_ID,uint32_t* DLC, uint32_t* Rx_DATA){

		uint8_t j;
		uint32_t dummyRead;

		/** Read CODE of the mailbox field */
		*Rx_CODE =  (can_pointer->RAMn[ MB_RX * MSG_BUF_SIZE + 0] & CODE_SHIFT_MASK) >> _24_SHIFT;
		/** Store the received ID */
		*Rx_ID =	(can_pointer->RAMn[ MB_RX * MSG_BUF_SIZE + 1] & CAN_WMBn_ID_ID_MASK) >> ID_MSG_SHIFT;
		/** Store the received DLC */
		*DLC = (can_pointer->RAMn[ MB_RX * MSG_BUF_SIZE + 0] & CAN_WMBn_CS_DLC_MASK) >> CAN_WMBn_CS_DLC_SHIFT;

		/** Store into the pointer the data in two words (8 bytes) */
		for (j = 0; j < 2; j++) {
			*Rx_DATA = can_pointer->RAMn[ MB_RX * MSG_BUF_SIZE + 2 + j];
			Rx_DATA++;
		}

		/** Dummy read of the Time Stamp */
		RxTIMESTAMP = (can_pointer->RAMn[0 * MSG_BUF_SIZE + 0] & TIMESTAMP_MASK);
		/** Dummy read of the timer to unlock message buffers */
		dummyRead = can_pointer->TIMER;
		/** Clear the 4th flag of the Message buffer  */
		can_pointer->IFLAG1 = FLAG1_MB4_MASK;


}
