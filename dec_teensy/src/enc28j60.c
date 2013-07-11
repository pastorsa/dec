/*
    Copyright (C) 2012 RedoX <dev@redox.ws>
    This file is part of TeenC.

        TeenC is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        TeenC is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with TeenC.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "common.h"
#include "enc28j60.h"
#include "spi.h"

uint16_t ENC_PckRx(uint8_t *b,uint16_t l) {
	uint16_t pl,ps;											  /* RxD Packet Length, Packet Status */
	if(ENC_hasRxd) {
		ENC_RegWrite(ERDPTL,(ENC_RxNxt)&0xFF);		 /* Set the Read Pointer to ENC_RxNxt Address */
		ENC_RegWrite(ERDPTH,(ENC_RxNxt)>>8);
		ENC_RxNxt = ENC_Read(ENC_RBM,0); 		 /* Update ENC_RxNxt with the next Packet Address */
		ENC_RxNxt |= ENC_Read(ENC_RBM,0)<<8;
		pl = ENC_Read(ENC_RBM,0)-4; 						  /* Get (Packet-CRC) Length, 2 bytes */
		pl |= ENC_Read(ENC_RBM,0)<<8;
		ps = ENC_Read(ENC_RBM,0); 									/* Get Packet Status, 2 bytes */
		ps |= ENC_Read(ENC_RBM,0)<<8;
		//dbg_s("\nPckLen: ");dbg_n16(pl);dbg_c('\n');
		//dbg_s("\nPckStat: ");dbg_n16(ps);dbg_c('\n');
		if(ps&0x80) {				  /* Check in Status if Received Ok (CRC and no symbol error) */
			if(pl+1>l)											/* Adjust PacketLen to Buffer Len */
				pl=l;
			ENC_BMRead(b,pl);										   /* Fill b with Packet Data */
			l = (ENC_RxNxt && ENC_RxStart) ? ENC_RxStop : ENC_RxNxt-1; 			/* Adjust Address */
			//dbg_n((l)>>8); dbg_n((l)&0xFF);
			ENC_RegWrite(ERXRDPTL,(l)&0xFF);	 /* Free Memory in ENC RAM  - Update Read Pointer */
			ENC_RegWrite(ERXRDPTH,(l)>>8);
		} else /* Discard Packet */
			pl = 0;
		ENC_Write(ENC_BfS, ECON2, ECON2_PKTDEC);					  /* Decrement Packet Counter */
	} else /* No Packet Available */
		pl = 0;
	return pl;															  /* Return Packet Length */
}

void ENC_PckTx(uint8_t *b,uint16_t l) {
	while(ENC_RegRead(ECON1) & ECON1_TXRTS) 				   /* Wait for Previous Tx Completion */
		/* Errata Rev7 - Transmit Logic
	     * Wait for Tx Error Interrupt, then Reset TxLogic
	     */
		if(ENC_RegRead(EIR) &  EIR_TXERIF) {
			ENC_Write(ENC_BfS,ECON1,ECON1_TXRST); 								   /* Start Reset */
			ENC_Write(ENC_BfC,ECON1,ECON1_TXRST); 									 /* End Reset */
			ENC_Write(ENC_BfC,EIR,EIR_TXERIF);    						  /* Clear Interrupt Flag */
		}

	ENC_RegWrite(EWRPTL,ENC_TxStart&0xFF);						  /* Set Write Pointer to TxStart */
	ENC_RegWrite(EWRPTH,ENC_TxStart>>8);
	/* Update TxEnd Addr just after Data, let space for TxVector */
	ENC_RegWrite(ETXNDL,(ENC_TxStart+l)&0xFF);
	ENC_RegWrite(ETXNDH,(ENC_TxStart+l)>>8);
	ENC_Write(ENC_WBM,0,0);		   /* Write Control Byte to 0x0, make the ENC use MACON3 settings */
	ENC_BMWrite(b,l);											   /* Write Data in Memory Buffer */
	ENC_Write(ENC_BfC,EIR,EIR_TXERIF|EIR_TXIF);							/* Clear Interrupts Flags */
	ENC_Write(ENC_BfS,ECON1,ECON1_TXRTS);					   /* Send Packet - Enable Transmit - */
	while(!(ENC_RegRead(EIR) & (EIR_TXERIF|EIR_TXIF)));		   /* Wait for Tx Completion or Error */
	ENC_Write(ENC_BfC,ECON1,ECON1_TXRTS);									  /* Disable Transmit */
	/* *** BEWARE not to Set ERDPT to an even Address *** */
	//~ ENC_RegWrite(ERDPTL,(ENC_TxStart+l+3)&0xFF);	 /* Set Read Pointer to ETXND+3 to get Status */
	//~ ENC_RegWrite(ERDPTL,(ENC_TxStart+l+3)>>8);
	//~  = ENC_Read(ENC_RBM,0); 							   /* Low Bit for truncated Status Vector */
	//~ if(l&0x80) {													  /* Check if Tx Error, retry */
	//~ 	ENC_Write(ENC_BfS,ECON1,ECON1_TXRST); 									 /* TxReset Start */
	//~ 	ENC_Write(ENC_BfC,ECON1,ECON1_TXRST); 									   /* TxReset End */
	//~ 	ENC_Write(ENC_BfC,EIR,EIR_TXERIF|EIR_TXIF);						/* Clear Interrupts Flags */
	//~ 	ENC_Write(ENC_BfS,ECON1,ECON1_TXRTS);				   /* Send Packet - Enable Transmit - */
	//~ 	//dbg_s("Err : "); dbg_n(st); dbg_c('\n');
	//~ }
}

void ENC_PhyWrite(uint8_t Addr, uint16_t d) {
	ENC_RegWrite(MIREGADR, Addr); 													  /* Set Addr */
    ENC_RegWrite(MIWRL, d&0xFF); 												/* Write Data Low */
    ENC_RegWrite(MIWRH, d>>8); 												   /* Write Data High */
    while(ENC_RegRead(MISTAT) & MISTAT_BUSY); 						  /* Wait until PHY completed */
}

uint8_t ENC_PhyReadH(uint8_t Addr) {
	ENC_RegWrite(MIREGADR, Addr); 													  /* Set Addr */
	ENC_RegWrite(MICMD, MICMD_MIIRD); 											 /* Start Read Op */
	while(ENC_RegRead(MISTAT) & MISTAT_BUSY) 						  /* Wait until PHY completed */
		_delay_us(15);
	ENC_RegWrite(MICMD, 0); 													/* Reset Read Bit */
	return ENC_RegRead(MIRDH);												  /* Return High Byte */
}

void ENC_Write(uint8_t Op, uint8_t Addr, uint8_t c) {
	ChS; 																		   /* Chip Select */
	SPI_Tx(Op|(AddrMsk & Addr)); 										  /* SPI Send Op and Addr */
	SPI_Tx(c); 																	 /* SPI Send Data */
	ChD;																		 /* Chip Deselect */
}

uint8_t ENC_Read(uint8_t Op, uint8_t Addr) {
	ENC_SBank((Addr&BankMsk)>>5);										 /* Change Bank if Needed */
	ChS;																		   /* Chip Select */
	SPI_Tx(Op|(AddrMsk & Addr)); 									   /* SPI Send Read Operation */
	if(Addr&0x80)									  /* SPI Dummy Read if Needed for PHY and MAC */
		SPI_Rx();
	Addr = SPI_Rx(); 															 /* SPI Data Read */
	ChD;																		 /* Chip Deselect */
	return Addr;															 /* Return Read Value */
}

void ENC_BMWrite(uint8_t *b,uint16_t l) {
	ChS;																		   /* Chip Select */
	SPI_Tx(ENC_WBM);												  /* SPI Send Buffer Write Op */
	while(l--)													   /* For each byte of the Buffer */
		SPI_Tx(*(b++));													   /* SPI Send Byte in BM */
	ChD;																		 /* Chip Deselect */
}

void ENC_BMRead(uint8_t *b,uint16_t l) {
	ChS;																		   /* Chip Select */
	SPI_Tx(ENC_RBM);												   /* SPI Send Buffer Read Op */
	while(l--)														   /* While Buffer's not full */
		*(b++) = SPI_Rx();												  /* Fill it with Rx Byte */
	ChD;																		 /* Chip Deselect */
}

void ENC_SBank(uint8_t b) {
	if(b != ENC_Bank) {
		//dbg_s("\nGo Bank "); dbg_n(b); dbg_c('\n');
		ENC_Bank=b;											 			   /* Update current Bank */
		ENC_Write(ENC_BfC, ECON1, (ECON1_BSEL1|ECON1_BSEL0)); 				   /* Clear Bank Bits */
		ENC_Write(ENC_BfS, ECON1, b);						 					 /* Set Bank Bits */
	}
}

void ENC_Init(uint8_t Mac[6]) {
	ENC_Bank=4; 										/* Init Bank Number to an impossible Value*/
	ENC_SoftRst; 														   /* ENC28J60 Soft Reset */
	_delay_ms(10); 													   /* Reset Delay - cf Errata */

	/* Bank 0 - Set Buffer Memory Registers */
	ENC_RegWrite(ERXSTL, ENC_RxStart&0xFF); 									/* Rx Start - End */
	ENC_RegWrite(ERXSTH, ENC_RxStart>>8);
	ENC_RegWrite(ERXNDL, ENC_RxStop&0xFF);
	ENC_RegWrite(ERXNDH, ENC_RxStop>>8);
	ENC_RegWrite(ERXRDPTL, ENC_RxStart&0xFF);							   /* RxRead Pointer Addr */
	ENC_RegWrite(ERXRDPTH, ENC_RxStart>>8);
	ENC_RegWrite(ETXSTL, ENC_TxStart&0xFF);										/* Tx Start - End */
	ENC_RegWrite(ETXSTH, ENC_TxStart>>8);
	ENC_RegWrite(ETXNDL, ENC_TxStop&0xFF);
	ENC_RegWrite(ETXNDH, ENC_TxStop>>8);
	/* Bank 1 - Packet Filtering -
	 * Reject Bad CRC
	 * For broadcast packets we allow only ARP packtets
     * All other packets should be unicast for our mac (MAADR)
     *
     * The pattern to match on is therefore
     * Type     ETH.DST
     * ARP      BROADCAST
     * 06 08 -- ff ff ff ff ff ff -> ip checksum for theses bytes=f7f9
     * in binary these poitions are:11 0000 0011 1111
     * This is hex 303F->EPMM0=0x3f,EPMM1=0x30
	 */
	ENC_RegWrite(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN);
	ENC_RegWrite(EPMM0, 0x3f);
	ENC_RegWrite(EPMM1, 0x30);
	ENC_RegWrite(EPMCSL, 0xf9);
	ENC_RegWrite(EPMCSH, 0xf7);
	/* Bank 2 - MAC Related Stuff -
	 * Enable MAC receive, Full Duplex IEEE flow control possible
	 * Automatic 60 bytes Padding, CRC Operations
	 * IEEE 803.2 Standard conformity - Half Duplex -
	 * Maximum Packet Size accepted
	 * Back-to-Back Inter-Packet Gap : 0x12->Half Duplex, 0x15->Full
	 * Non-Back-to-Back Inter-Packet Gap Low/High to 0x12 and 0xC
	 */
	ENC_RegWrite(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
	ENC_RegWrite(MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
	ENC_RegWrite(MACON4, MACON4_DEFER);
	ENC_RegWrite(MAMXFLL, ENC_MaxFrameLen&0xFF);
	ENC_RegWrite(MAMXFLH, ENC_MaxFrameLen>>8);
	ENC_RegWrite(MABBIPG, 0x12);
	ENC_RegWrite(MAIPGL, 0x12);
	ENC_RegWrite(MAIPGH, 0xC);
	/* Bank 3 - More MAC Related Stuff -
	 * Set MAC Address
	 */
	ENC_RegWrite(MAADR6, Mac[5]);
	ENC_RegWrite(MAADR5, Mac[4]);
	ENC_RegWrite(MAADR4, Mac[3]);
	ENC_RegWrite(MAADR3, Mac[2]);
	ENC_RegWrite(MAADR2, Mac[1]);
	ENC_RegWrite(MAADR1, Mac[0]);
	/* PHY Init
	 * No Loopback
	 */
	ENC_PhyWrite(PHCON2, PHCON2_HDLDIS);
	ENC_Write(ENC_BfS, EIE, EIE_INTIE|EIE_PKTIE);	   /* Enable Interrupts and Packets Reception */
	ENC_Write(ENC_BfS, ECON1, ECON1_RXEN);
}
