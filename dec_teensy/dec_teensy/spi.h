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

#ifndef _SPI_h
#define _SPI_h

#include <avr/io.h>

#define SPI_Port PORTB
#define SPI_PortD DDRB
#define SPI_CS PB0 										/* Slave active: 0 */
#define SPI_SCLK PB1
#define SPI_MOSI PB2
#define SPI_MISO PB3

#define ChS SPI_Port &= ~_BV(SPI_CS)				/* Chip Select */
#define ChD SPI_Port |= _BV(SPI_CS)						/* Deselect */

#define SPI_DP2 1 				 /* Default Prescaler: fspi = fosc/2 */
#define SPI_DP4 0 				 /* Default Prescaler: fspi = fosc/4 */

/* Additionnal prescaler */
#define SPI_PRESC_1 0 													/* fspi */
#define SPI_PRESC_4 _BV(SPR0) 									 /* fspi/4 */
#define SPI_PRESC_16 _BV(SPR1) 							   /* fspi/16 */
#define SPI_PRESC_32 _BV(SPR1)|_BV(SPR0) 			   /* fspi/32 */

void SPI_Init(unsigned char hof,unsigned char presc);

void SPI_Send(unsigned char c);
unsigned char SPI_Recv(void);

void SPI_Tx(unsigned char b);
unsigned char SPI_Rx(void);

#endif
