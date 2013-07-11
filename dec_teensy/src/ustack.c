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
#include "ustack.h"

/* Return Length of Written Data */
uint8_t BuffWrite(uint8_t *b,char *s) {
	char c=0x1;
	uint8_t l=0;
	while(c) {
		c = *(s++);
		*(b++) = c;
		l++;
	}
	return l;
}

/* Return Length of Written Data */
uint8_t BuffWritePM(uint8_t *b,char *s) {
	char c=0x1;
	uint8_t l=0;
	while(c) {
		c = pgm_read_byte(s++);
		*(b++) = c;
		l++;
	}
	return l;
}

void IP_Send(uint8_t *b, uint8_t hl, uint16_t dl,uint8_t m) {
	*(b+IP_Start) = (IP_Vers<<4)|(hl>>2);								  /* Update Header Length */
	*(b+IP_Total_Len_H) = (hl+dl)>>8;									   /* Update Total Length */
	*(b+IP_Total_Len_L) = (hl+dl)&0xFF;
	if(m&ModeReply) {														   /* If it's a Reply */
		EthH_Reply(b);													  /* Switch MAC Addresses */
		/* We dont need dl anymore, reuse it */
		for(dl=0;dl<EthIPLen;dl++) {										/* Update Dest/Src Ip */
			*(b+IP_DestIP+dl) = *(b+IP_SrcIP+dl);
			*(b+IP_SrcIP+dl) = IPAddr[dl];
		}
	} else {
		*(b+IP_Checksum_H)= 0;													/* Erase Checksum */
		*(b+IP_Checksum_L)= 0;
	}
	*(b+IP_Flags_H) = IP_FlagDF;										/* Don't Fragment Flag ON */
	*(b+IP_Flags_L) = 0x0;
	*(b+IP_TTL) = IP_TTL_v;															   /* Set TTL */
	dl = IP_Checksum(b+IP_Start,hl);										  /* Compute Checksum */
	*(b+IP_Checksum_H) = dl>>8;												   /* Update Checksum */
	*(b+IP_Checksum_L) = dl&0xFF;
}

void TCP_Send(uint8_t *b,uint8_t hl,TCP_Data *tcp_s,uint8_t m) {
	uint16_t chks;
	IP_Send(b,hl,tcp_s->HLen+tcp_s->DLen,m);									/* Make IP Header */
	b += IP_Start+hl;
	if(m&ModeReply) {														   /* If it's a Reply */
		m = *(b+TCP_SrcPort_H);													  /* Switch Ports */
		*(b+TCP_SrcPort_H) = *(b+TCP_DestPort_H);
		*(b+TCP_DestPort_H) = m;
		m = *(b+TCP_SrcPort_L);
		*(b+TCP_SrcPort_L) = *(b+TCP_DestPort_L);
		*(b+TCP_DestPort_L) = m;
		for(m=0;m<4;m++) {														/* Update Seq/Ack */
			chks = *(b+TCP_AckN+m);													  /* Save Ack */
		 	*(b+TCP_AckN+m) = *(b+TCP_SeqN+m);									 /* Mv Seq To Ack */
		 	*(b+TCP_SeqN+m) = chks;												 /* Mv Acq to Seq */
		}
		/* Should Use uint32_t but assume we don't receive more than 255 bytes at a time */
		*(b+TCP_AckN+0x3)+=tcp_s->Ss;										 /* Update Seq Number */
		*(b+TCP_SeqN+0x3)+=tcp_s->As;
		*(b+TCP_Flags) = tcp_s->Flags;										      /* Update Flags */
	} else {
		*(b+TCP_Flags) = TCP_fPSH|TCP_fACK;								/* Update Flags to PSH/ACK*/
		if(m&ModeSingle)
			*(b+TCP_Flags) |= TCP_fFIN;								/* If single Packet, send FIN */
	}
	*(b+TCP_HLen) = (tcp_s->HLen>>2)<<4;									  /* Update HeaderLen */
	*(b+TCP_Checksum_H) = 0x0;													/* Clear Checksum */
	*(b+TCP_Checksum_L) = 0x0;
	chks = UDP_TCP_Checksum(b-IP_Start-hl,hl);								  /* Compute Checksum */
	*(b+TCP_Checksum_H) = chks>>8;														/* Update */
	*(b+TCP_Checksum_L) = chks&0xFF;
	b -= IP_Start+hl;
	//dbg_s("\nSend:\n");
	//for(chks = 0;chks<EthLen+hl+tcp_s->HLen+tcp_s->DLen;chks++) {
		//dbg_n(*(b+chks)); dbg_c(' ');
	//}
	Send_Pck(b,EthLen+hl+tcp_s->HLen+tcp_s->DLen);								   /* Send Packet */
}

/* Return 1 if it's an valid IP Packet, else 0 */
uint8_t TCP_Recv(uint8_t *b,uint8_t hl,uint16_t l,TCP_Data *tcp_s) {
	uint16_t chks_p=0;
	b+= IP_Start+hl;
	#ifdef _UDP_TCP_DCheck
	chks_p = *(b+TCP_Checksum_H)<<8;								  /* Get Checksum from Packet */
	chks_p |= *(b+TCP_Checksum_L);
	//dbg_s("\nPacket Checksum: ");dbg_n16(chks_p);
	#endif
	*(b+TCP_Checksum_H) = 0x0;													/* Erase Checksum */
	*(b+TCP_Checksum_L) = 0x0;
	//dbg_s("\nComputed Checksum: ");dbg_n16(UDP_TCP_Checksum(b,hl));
	#ifdef _UDP_TCP_DCheck
	if(UDP_TCP_Checksum(b-(IP_Start+hl),hl) == chks_p) {			/* Check if checksum is valid */
	#endif
		tcp_s->Flags = *(b+TCP_Flags);											 /* Get TCP Flags */
		tcp_s->HLen = (*(b+TCP_HLen)>>4)*4;								   /* Get TCP Data Offset */
		b-=(IP_Start+hl);

		chks_p = IP_GetDataLen(b,hl);									   /* Get TCP Data Length */

		if(chks_p > l - IP_Start - hl - tcp_s->HLen)   /* If packet is truncated, avoid overflow  */
			chks_p = l - IP_Start - hl - tcp_s->HLen;
		tcp_s->DLen = chks_p;												   /* Update Data Len */
		chks_p = 1;
	#ifdef _UDP_TCP_DCheck
	} else
		chks_p = 0;
	#endif
	return chks_p;																 /* Return Status */
}

void UDP_Send(uint8_t *b,uint8_t hl, uint16_t dl,uint8_t m) {
	uint16_t chks;
	IP_Send(b,hl,UDP_HeaderLen+dl,m);										  /* Update IP Header */
	b+= IP_Start+hl;
	if(m&ModeReply) {														   /* If it's a Reply */
		chks = *(b+UDP_SrcPort_H);												  /* Switch Ports */
		*(b+UDP_SrcPort_H) = *(b+UDP_DestPort_H);
		*(b+UDP_DestPort_H) = chks;
		chks = *(b+UDP_SrcPort_L);
		*(b+UDP_SrcPort_L) = *(b+UDP_DestPort_L);
		*(b+UDP_DestPort_L) = chks;
	}

	*(b+UDP_LenH) = (UDP_HeaderLen+dl)>>8;										/* Update UDP Len */
	*(b+UDP_LenL) = (UDP_HeaderLen+dl)&0xFF;
	chks = UDP_TCP_Checksum(b-(IP_Start+hl),hl);							  /* Compute Checksum */
	*(b+UDP_Checksum_H) = chks>>8;														/* Add It */
	*(b+UDP_Checksum_L) = chks&0xFF;
	b -= IP_Start+hl;
	/* for(i = 0;i<EthLen+hl+UDP_HeaderLen+dl;i++) { dbg_n(*(b+i)); dbg_c(' '); }*/
	Send_Pck(b,EthLen+UDP_HeaderLen+hl+dl);										   /* Send Packet */
}

/* Return Pseudo Checksum */
uint16_t UDP_TCP_Checksum(uint8_t *b, uint8_t hl) {
	uint32_t s = 0;
	uint16_t i;
	b += IP_SrcIP;
	for(i=0;i<EthIPLen;i++) {												   /* Add Src/Dest IP */
		s += ((uint16_t)(*b)<<8)|*(b+1);
		b+=2;
	}
	b -= IP_SrcIP+2*EthIPLen;							/* Go back to the beggining of the buffer */
	s += *(b+IP_Protocol);													   /* Add IP protocol */
	i = (uint16_t)IP_GetDataLen(b,hl);										/* Get UDP/TCP Length */
	s +=i;																			/* Add to Sum */
	/* dbg_s("\nDataLen: ");dbg_n16(IP_GetDataLen(b,hl));dbg_c('\n'); */
	b += IP_Start+hl;									    /* Go to UDP/TCP Header Start Address */
	while(i > 1)  {									 /* Compute Checksum of UDP/TCP Header + Data */
		s += (((uint16_t)(*b))<<8)|*(b+1);
		i -= 2;
		b +=2;
   }
   if(i> 0)
		   s += ((uint16_t)(*b))<<8;
   while (s>>16)
	   s = (s & 0xFFFF) + (s >> 16);
   return ~((uint16_t)s);	 												   /* Return Checksum */
}

/* Return UDP Data Length */
uint16_t UDP_Recv(uint8_t *b, uint8_t hl, uint16_t l) {
	uint16_t chks_p;
	#ifdef _UDP_TCP_DCheck
	chks_p = *(b+IP_Start+hl+UDP_Checksum_H)<<8;					  /* Get Checksum from Packet */
	chks_p |= *(b+IP_Start+hl+UDP_Checksum_L);
	//dbg_s("\nPacket Checksum: ");dbg_n16(chks_p);
	#endif
	*(b+IP_Start+hl+UDP_Checksum_H) = 0x0;									/* Erase UDP Checksum */
	*(b+IP_Start+hl+UDP_Checksum_L) = 0x0;
	#ifdef _UDP_TCP_DCheck
	//dbg_s("\nComputed Checksum: ");dbg_n16(UDP_TCP_Checksum(b,hl));
	if(UDP_TCP_Checksum(b,hl) == chks_p) {							/* Check if checksum is valid */
	#endif
		chks_p = IP_GetDataLen(b,hl)-UDP_HeaderLen;	   						  /* Get UDP Data Len */
		if( chks_p > l - IP_Start - hl - UDP_HeaderLen)/* If packet is truncated, avoid overflow  */
			chks_p = l - IP_Start - hl - UDP_HeaderLen;
	#ifdef _UDP_TCP_DCheck
	} else
		chks_p =0;
	#endif
	return chks_p;															   /* Return Data Len */
}

/* Return IP"Data Length */
uint16_t IP_GetDataLen(uint8_t *b,uint8_t hl) {
	return (uint16_t)(((*(b+IP_Total_Len_H)<<8) | (*(b+IP_Total_Len_L))) - hl);
}

void ICMP_Reply(uint8_t *b,uint8_t hl, uint16_t l) {
	uint16_t chks_p,dl;
	if(*(b+IP_Start+hl) == ICMP_tReq) {							/* If ICMP Request (Echo), accept */
		dl = IP_GetDataLen(b,hl);
		if(dl>l-EthLen-IP_HeaderLen)		/* Avoid Buffer Overflow if packet has been truncated */
			dl = l-EthLen-IP_HeaderLen;
		chks_p = *(b+IP_Start+hl+ICMP_Checksum_H)<<8;						 /* Get ICMP Checksum */
		chks_p |= *(b+IP_Start+hl+ICMP_Checksum_L);
		*(b+IP_Start+hl+ICMP_Checksum_H) = 0;									/* Erase checksum */
		*(b+IP_Start+hl+ICMP_Checksum_L) = 0;
		if(chks_p == IP_Checksum(b+IP_Start+hl,dl)) {
			IP_Send(b,hl,dl,ModeReply);								   /* Checksum Ok, Echo Reply */
			*(b+ICMP_Start) = ICMP_tRep;								/* Set type to Echo-reply */
			*(b+ICMP_Start+1) = 0;											   /* Reset ICMP Code */
			chks_p = IP_Checksum(b+ICMP_Start,dl);							  /* Compute Checksum */
			//dbg_s(" Computed Checksum: "); dbg_n16(chks_p); dbg_c('\n');
			*(b+ICMP_Start+ICMP_Checksum_H) = chks_p>>8;					   /* Update Checksum */
			*(b+ICMP_Start+ICMP_Checksum_L) = chks_p&0xFF;
			Send_Pck(b,EthLen+IP_HeaderLen+dl);										/* Send Reply */
		}
	}
}

/* Return IP Checksum */
uint16_t IP_Checksum(uint8_t *b,uint16_t l) {
   uint32_t s = 0;
	while(l > 1)  {
		s += (((uint16_t)(*b))<<8)|*(b+1);
		l -= 2;
		b +=2;
   }
	if(l > 0)
		s += ((uint16_t)*b)<<8;
	while (s>>16)
		s = (s&0xFFFF)+(s>>16);
	return ~((uint16_t)s);
}

/* Return 0 if Invalid Packet, or IP Header Length if it's a valid one */
uint8_t IP_Check(uint8_t *b, uint16_t l) {
	uint16_t chks_p;
	uint8_t hlen;
	hlen = (*(b+IP_Start)&0xF)*4;										   /* Header Len in bytes */
	chks_p = *(b+IP_Protocol);									   /* Check if Protocol supported */
	if(chks_p == IP_pICMP || chks_p == IP_pUDP || chks_p == IP_pTCP) {
		chks_p = (*(b+IP_Checksum_H))<<8;										  /* Get Checksum */
		chks_p |= *(b+IP_Checksum_L);
		*(b+IP_Checksum_H)= 0;												 /* Erase from Packet */
		*(b+IP_Checksum_L)= 0;
		if(chks_p!=IP_Checksum((b+IP_Start),hlen)) 			/* If checksum Invalid, discard packet*/
			hlen=0;
	} else
		hlen = 0;																/* Discard Packet */
	return hlen;
}

/* Return 1 if packet was arp, else 0 */
uint8_t ifARP_Reply(uint8_t *b, uint16_t l) {
	uint8_t i;
	for(i=0;i<EthIPLen;i++)													  /* Is for this IP ? */
		if(*(b+ARP_DestIP+i) != IPAddr[i])
			i=0xFE;

	if(i!=0xFF) {
		if(*(b+ARP_Op_H) == ARP_Op_Req_H && *(b+ARP_Op_L) == ARP_Op_Req_L) {	/* Is ARP Request */
			EthH_Reply(b);												 /* Well, make ARP Answer */

			*(b+ARP_Op_L) = ARP_Op_Rep_L;										 /* Update ARP Op */
			for(i=0;i<EthMacLen;i++) {								   /* Update ARP Dest/Src MAC */
				*(b+ARP_DestMac+i) = *(b+i);
				*(b+ARP_SrcMac+i) = MACAddr[i];
			}
			for(i=0;i<EthIPLen;i++) {									/* Update ARP Dest/Src IP */
				*(b+ARP_DestIP+i) = *(b+ARP_SrcIP+i);
				*(b+ARP_SrcIP+i) = IPAddr[i];
			}
			Send_Pck(b,l);											 /* Send Reply on the Network */
			i=1; 												/* Update Status - Packet Handeld */
		}
	} else
		i++;																   /* Set Status to 0 */
	return i;																	 /* Return Status */
}

void EthH_Reply(uint8_t *b) {
	uint8_t i;
	for(i=0;i<EthMacLen;i++) {			/* Update Ethernet Header for reply (switch Src/Dest MAC) */
		*(b+i) = *(b+i+EthMacLen);
		*(b+EthMacLen+i) = MACAddr[i];
	}
}