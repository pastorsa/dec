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

#ifndef _uStack_h
#define _uStack_h

#include <avr/pgmspace.h>

/* Ethernet Header*/
#define EthLen 0xE

#define EthType_H 0xC
#define EthType_L 0xD

#define EthTypeARP_H 0x8
#define EthTypeARP_L 0x6

#define EthTypeIP_H 0x8
#define EthTypeIP_L 0x0

#define EthMacLen 0x6
#define EthIPLen 0x4

/* ARP Packet */
#define ARP_Start 0xE

#define ARP_Op_H (ARP_Start+0x6)
#define ARP_Op_L (ARP_Start+0x7)

#define ARP_Op_Req_H 0x00
#define ARP_Op_Req_L 0x01

#define ARP_Op_Rep_H 0x00
#define ARP_Op_Rep_L 0x02

#define ARP_SrcMac (ARP_Start+0x8)
#define ARP_SrcIP (ARP_Start+0xE)

#define ARP_DestMac (ARP_Start+0x12)
#define ARP_DestIP (ARP_Start+0x18)

/* IPv4 Packet */
#define IP_Start 0xE
#define IP_Vers 0x4

#define IP_HeaderLen 0x14

#define IP_Total_Len_H (IP_Start+0x2)
#define IP_Total_Len_L (IP_Start+0x3)

#define IP_Ident_H (IP_Start+0x4)
#define IP_Ident_L (IP_Start+0x5)

#define IP_Flags_H (IP_Start+0x6)
#define IP_Flags_L (IP_Start+0x7)
#define IP_FlagDF 0b01000000

#define IP_TTL (IP_Start+0x8)
#define IP_TTL_v 64

#define IP_Protocol (IP_Start+0x9)
#define IP_pICMP 0x1
#define IP_pUDP 0x11
#define IP_pTCP 0x6

#define IP_gProtocol(b) *(b+IP_Protocol)

#define IP_Checksum_H (IP_Start+0xA)
#define IP_Checksum_L (IP_Start+0xB)

#define IP_SrcIP (IP_Start+0xC)
#define IP_DestIP (IP_SrcIP+EthIPLen)

/* ICMP Packet */
 #define ICMP_Start (IP_Start+IP_HeaderLen)

#define ICMP_tReq 0x8
#define ICMP_tRep 0x0

#define ICMP_Checksum_H 0x2
#define ICMP_Checksum_L 0x3

#define ICMP_Data 0x4

/* UDP Packet */
#define UDP_Checksum_H 0x6
#define UDP_Checksum_L 0x7

#define UDP_SrcPort_H 0x0
#define UDP_SrcPort_L 0x1
#define UDP_DestPort_H 0x2
#define UDP_DestPort_L 0x3

#define UDP_LenH 0x4
#define UDP_LenL 0x5

#define UDP_HeaderLen 0x8

/* TCP Packet */
#define TCP_SrcPort_H 0x0
#define TCP_SrcPort_L 0x1
#define TCP_DestPort_H 0x2
#define TCP_DestPort_L 0x3

#define TCP_SeqN 0x4
#define TCP_AckN 0x8

#define TCP_HLen 0xC

#define TCP_Flags 0xD

#define TCP_fFIN  0x1
#define TCP_fSYN  0x2
#define TCP_fRST  0x4
#define TCP_fPSH  0x8
#define TCP_fACK  0x10
#define TCP_fURG  0x20
#define TCP_fECN0 0x40
#define TCP_fECH1 0x80

#define TCP_Checksum_H 0x10
#define TCP_Checksum_L 0x11

#define TCP_Opts 0x14

#define TCP_HeaderLen 0x14

#define Get16B(p) ((*(p)<<8)|*(p+1))

/* Functions Aliases */
#define Send_Pck ENC_PckTx
#define Recv_Pck ENC_PckRx

#define ModeReply 0x8
#define ModeSend 0x1
#define ModeSingle 0x2
#define ModeSingleSend (ModeSend|ModeSingle)

typedef struct {
	uint8_t Flags;
	uint8_t HLen;
	uint16_t DLen;
	uint8_t Ss;
	uint8_t As;
} TCP_Data;

void EthH_Reply(uint8_t *b);
void IP_Send(uint8_t *b, uint8_t hl, uint16_t dl,uint8_t m);
uint8_t IP_Check(uint8_t *b, uint16_t l) ;

uint16_t IP_GetDataLen(uint8_t *b,uint8_t hl);
uint16_t IP_Checksum(uint8_t *b,uint16_t l);

uint16_t UDP_TCP_Checksum(uint8_t *b, uint8_t hl);

uint8_t ifARP_Reply(uint8_t *b, uint16_t l);

void ICMP_Reply(uint8_t *b,uint8_t hl, uint16_t l);

uint16_t UDP_Recv(uint8_t *b, uint8_t hl, uint16_t l);
void UDP_Send(uint8_t *b,uint8_t hl, uint16_t dl,uint8_t m);

uint8_t TCP_Recv(uint8_t *b,uint8_t hl,uint16_t l,TCP_Data *tcp_s);
void TCP_Send(uint8_t *b,uint8_t hl,TCP_Data *tcp_s,uint8_t m);

uint8_t BuffWrite(uint8_t *b,char *s);
uint8_t BuffWritePM(uint8_t *b,char *s);

static const uint8_t IPAddr[4] = {10,0,0,2};
static uint8_t MACAddr[6] = {0x0,0x11,0x22,0x33,0x44,0x55};
#endif
