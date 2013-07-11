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
#include "spi.h"

void SPI_Init(unsigned char hof,unsigned char presc) {
	/* Set MOSI, SCK and CS output all others input */
	SPI_Port = _BV(SPI_MOSI)|_BV(SPI_SCLK)|_BV(SPI_CS);
	SPI_PortD = _BV(SPI_MOSI)|_BV(SPI_SCLK)|_BV(SPI_CS);
	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = _BV(SPE)|_BV(MSTR)|(presc);
	if(hof)
		SPSR |= _BV(SPI2X);					/* Fosc/2 if no prescaler */
}

void SPI_Tx(unsigned char c) {
	//dbgf_c('w'); dbgf_n(c); dbgf_c(' ');
	SPDR = c; 											 /* Start transmission */
	while(!(SPSR & (1<<SPIF)));			 /* Wait for Tx complete */
}

void SPI_Send(unsigned char c) {
	ChS; 																/* Chip Select */
	SPI_Tx(c);															  /* Send Byte */
	ChD; 															/* Chip Deselect */
}

unsigned char SPI_Rx(void) {
	SPDR=0;													  /* Clear Rx Value */
	while(!(SPSR & (1<<SPIF)));			/* Wait for Rx complete */
	//dbgf_c('r'); dbgf_n(SPDR); dbgf_c(' ');
	return SPDR;												 /* Return Value */
}

unsigned char SPI_Recv(void) {
	ChS;																   /* Chip Select */
	SPI_Rx();																   /* SPI Rx */
	ChD; 															/* Chip Deselect */
	return SPDR;											     /* Return Value */
}