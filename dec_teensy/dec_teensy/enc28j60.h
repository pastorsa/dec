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

#ifndef _ENC_h
#define _ENC_h

#include <util/delay.h>

volatile uint8_t ENC_Bank;
volatile uint16_t ENC_RxNxt;

/* ENC28J60 Control Registers
 * Control register definitions are a combination of address,
 * bank number, and Ethernet/MAC/PHY indicator bits.
 * - Register address        (bits 0-4)
 * - Bank number        (bits 5-6)
 * - MAC/PHY indicator        (bit 7)
 */
#define AddrMsk          0x1F
#define BankMsk          0x60
#define SprdMsk          0x80

/* Common Registers */
#define EIE              0x1B
#define EIR              0x1C
#define ESTAT            0x1D
#define ECON2            0x1E
#define ECON1            0x1F
/* Bank 0 */
#define ERDPTL           0x00
#define ERDPTH           0x01
#define EWRPTL           0x02
#define EWRPTH           0x03
#define ETXSTL           0x04
#define ETXSTH           0x05
#define ETXNDL           0x06
#define ETXNDH           0x07
#define ERXSTL           0x08
#define ERXSTH           0x09
#define ERXNDL           0x0A
#define ERXNDH           0x0B
#define ERXRDPTL         0x0C
#define ERXRDPTH         0x0D
#define ERXWRPTL         0x0E
#define ERXWRPTH         0x0F
#define EDMASTL          0x10
#define EDMASTH          0x11
#define EDMANDL          0x12
#define EDMANDH          0x13
#define EDMADSTL         0x14
#define EDMADSTH         0x15
#define EDMACSL          0x16
#define EDMACSH          0x17
/* Bank 1 */
#define EHT0             (0x00|0x20)
#define EHT1             (0x01|0x20)
#define EHT2             (0x02|0x20)
#define EHT3             (0x03|0x20)
#define EHT4             (0x04|0x20)
#define EHT5             (0x05|0x20)
#define EHT6             (0x06|0x20)
#define EHT7             (0x07|0x20)
#define EPMM0            (0x08|0x20)
#define EPMM1            (0x09|0x20)
#define EPMM2            (0x0A|0x20)
#define EPMM3            (0x0B|0x20)
#define EPMM4            (0x0C|0x20)
#define EPMM5            (0x0D|0x20)
#define EPMM6            (0x0E|0x20)
#define EPMM7            (0x0F|0x20)
#define EPMCSL           (0x10|0x20)
#define EPMCSH           (0x11|0x20)
#define EPMOL            (0x14|0x20)
#define EPMOH            (0x15|0x20)
#define EWOLIE           (0x16|0x20)
#define EWOLIR           (0x17|0x20)
#define ERXFCON          (0x18|0x20)
#define EPKTCNT          (0x19|0x20)
/* Bank 2 */
#define MACON1           (0x00|0x40|0x80)
#define MACON2           (0x01|0x40|0x80)
#define MACON3           (0x02|0x40|0x80)
#define MACON4           (0x03|0x40|0x80)
#define MABBIPG          (0x04|0x40|0x80)
#define MAIPGL           (0x06|0x40|0x80)
#define MAIPGH           (0x07|0x40|0x80)
#define MACLCON1         (0x08|0x40|0x80)
#define MACLCON2         (0x09|0x40|0x80)
#define MAMXFLL          (0x0A|0x40|0x80)
#define MAMXFLH          (0x0B|0x40|0x80)
#define MAPHSUP          (0x0D|0x40|0x80)
#define MICON            (0x11|0x40|0x80)
#define MICMD            (0x12|0x40|0x80)
#define MIREGADR         (0x14|0x40|0x80)
#define MIWRL            (0x16|0x40|0x80)
#define MIWRH            (0x17|0x40|0x80)
#define MIRDL            (0x18|0x40|0x80)
#define MIRDH            (0x19|0x40|0x80)
/* Bank 3 */
#define MAADR5           (0x00|0x60|0x80)
#define MAADR6           (0x01|0x60|0x80)
#define MAADR3           (0x02|0x60|0x80)
#define MAADR4           (0x03|0x60|0x80)
#define MAADR1           (0x04|0x60|0x80)
#define MAADR2           (0x05|0x60|0x80)
#define EBSTSD           (0x06|0x60)
#define EBSTCON          (0x07|0x60)
#define EBSTCSL          (0x08|0x60)
#define EBSTCSH          (0x09|0x60)
#define MISTAT           (0x0A|0x60|0x80)
#define EREVID           (0x12|0x60)
#define ECOCON           (0x15|0x60)
#define EFLOCON          (0x17|0x60)
#define EPAUSL           (0x18|0x60)
#define EPAUSH           (0x19|0x60)

/* PHY */
#define PHCON1           0x00
#define PHSTAT1          0x01
#define PHHID1           0x02
#define PHHID2           0x03
#define PHCON2           0x10
#define PHSTAT2          0x11
#define PHIE             0x12
#define PHIR             0x13
#define PHLCON           0x14

/* ENC28J60 ERXFCON Register Bits */
#define ERXFCON_UCEN     0x80
#define ERXFCON_ANDOR    0x40
#define ERXFCON_CRCEN    0x20
#define ERXFCON_PMEN     0x10
#define ERXFCON_MPEN     0x08
#define ERXFCON_HTEN     0x04
#define ERXFCON_MCEN     0x02
#define ERXFCON_BCEN     0x01
/* ENC28J60 EIE Register Bits */
#define EIE_INTIE        0x80
#define EIE_PKTIE        0x40
#define EIE_DMAIE        0x20
#define EIE_LINKIE       0x10
#define EIE_TXIE         0x08
#define EIE_WOLIE        0x04
#define EIE_TXERIE       0x02
#define EIE_RXERIE       0x01
/* ENC28J60 EIR Register Bits */
#define EIR_PKTIF        0x40
#define EIR_DMAIF        0x20
#define EIR_LINKIF       0x10
#define EIR_TXIF         0x08
#define EIR_WOLIF        0x04
#define EIR_TXERIF       0x02
#define EIR_RXERIF       0x01
/* ENC28J60 ESTAT Register Bits */
#define ESTAT_INT        0x80
#define ESTAT_LATECOL    0x10
#define ESTAT_RXBUSY     0x04
#define ESTAT_TXABRT     0x02
#define ESTAT_CLKRDY     0x01
/* ENC28J60 ECON2 Register Bits */
#define ECON2_AUTOINC    0x80
#define ECON2_PKTDEC     0x40
#define ECON2_PWRSV      0x20
#define ECON2_VRPS       0x08
/* ENC28J60 ECON1 Register Bits */
#define ECON1_TXRST      0x80
#define ECON1_RXRST      0x40
#define ECON1_DMAST      0x20
#define ECON1_CSUMEN     0x10
#define ECON1_TXRTS      0x08
#define ECON1_RXEN       0x04
#define ECON1_BSEL1      0x02
#define ECON1_BSEL0      0x01

/* ENC28J60 MACON1 Register Bits */
#define MACON1_LOOPBK    0x10
#define MACON1_TXPAUS    0x08
#define MACON1_RXPAUS    0x04
#define MACON1_PASSALL   0x02
#define MACON1_MARXEN    0x01
/* ENC28J60 MACON2 Register Bits */
#define MACON2_MARST     0x80
#define MACON2_RNDRST    0x40
#define MACON2_MARXRST   0x08
#define MACON2_RFUNRST   0x04
#define MACON2_MATXRST   0x02
#define MACON2_TFUNRST   0x01
/* ENC28J60 MACON3 Register Bits */
#define MACON3_PADCFG2   0x80
#define MACON3_PADCFG1   0x40
#define MACON3_PADCFG0   0x20
#define MACON3_TXCRCEN   0x10
#define MACON3_PHDRLEN   0x08
#define MACON3_HFRMLEN   0x04
#define MACON3_FRMLNEN   0x02
#define MACON3_FULDPX    0x01
/* ENC28J60 MACON4 Register Bits */
#define MACON4_NOBKOFF   0x10
#define MACON4_BPEN      0x20
#define MACON4_DEFER     0x40

/* ENC28J60 MICMD Register Bits */
#define MICMD_MIISCAN    0x02
#define MICMD_MIIRD      0x01
/* ENC28J60 MISTAT Register Bits */
#define MISTAT_NVALID    0x04
#define MISTAT_SCAN      0x02
#define MISTAT_BUSY      0x01

/* ENC28J60 PHY PHCON1 Register Bits */
#define PHCON1_PRST      0x8000
#define PHCON1_PLOOPBK   0x4000
#define PHCON1_PPWRSV    0x0800
#define PHCON1_PDPXMD    0x0100
/* ENC28J60 PHY PHSTAT1 Register Bits */
#define PHSTAT1_PFDPX    0x1000
#define PHSTAT1_PHDPX    0x0800
#define PHSTAT1_LLSTAT   0x0004
#define PHSTAT1_JBSTAT   0x0002
/* ENC28J60 PHY PHCON2 Register Bits */
#define PHCON2_FRCLINK   0x4000
#define PHCON2_TXDIS     0x2000
#define PHCON2_JABBER    0x0400
#define PHCON2_HDLDIS    0x0100

/* ENC28J60 Packet Control Byte Bits */
#define PKTCTRL_PHUGEEN  0x08
#define PKTCTRL_PPADEN   0x04
#define PKTCTRL_PCRCEN   0x02
#define PKTCTRL_POVERRIDE 0x01

/* SPI Instruction Set */
#define ENC_RCR       0x00 												 /* Read Control Register */
#define ENC_WCR       0x40												/* Write Control Register */
#define ENC_RBM       0x3A													/* Read Buffer Memory */
#define ENC_WBM       0x7A												   /* Write Buffer Memory */
#define ENC_BfS       0x80														  /* Bitfield Set */
#define ENC_BfC       0xA0													    /* Bitfield Clear */

#define ENC_SoftRst   SPI_Send(0xFF)												/* Soft Reset */

/* RX buffer starts at 0x0 (Errata Rev7) / */
#define ENC_RxStart       0x0
/* Start TX buffer at 0x1FFF-0x0600,  one full ethernet frame (~1500 bytes) */
#define ENC_TxStart		 (0x1FFF-0x0600)
/* RX buffer ends before TxStart - Odd Addr -*/
#define ENC_RxStop    (ENC_TxStart-1)
/* TX buffer end: end of mem */
#define ENC_TxStop      0x1FFF

/* Max Frame Length accepted */
#define ENC_MaxFrameLen     1500

void ENC_Init(uint8_t Mac[6]);

void ENC_Write(uint8_t Op, uint8_t Addr, uint8_t c);
uint8_t ENC_Read(uint8_t Op, uint8_t Addr);

void ENC_BMWrite(uint8_t *b,uint16_t l);
void ENC_BMRead(uint8_t *b,uint16_t l);

void ENC_PhyWrite(uint8_t Addr, uint16_t d);
uint8_t ENC_PhyReadH(uint8_t Addr);

void ENC_SBank(uint8_t b);

void ENC_PckTx(uint8_t *b,uint16_t l);
uint16_t ENC_PckRx(uint8_t *b,uint16_t  l);

#define ENC_RegWrite(a,d) ENC_SBank((a&BankMsk)>>5); ENC_Write(ENC_WCR,a,d)
#define ENC_RegRead(a) ENC_Read(ENC_RCR,a)

#define ENC_hasRxd (ENC_RegRead(EPKTCNT)>0?1:0)

#define ENC_LinkSt (ENC_PhyReadH(PHSTAT2)&&0x4)
#define ENC_LEDInit ENC_PhyWrite(PHLCON,0x476)
#define ENC_RevID ENC_RegRead(EREVID)

#endif