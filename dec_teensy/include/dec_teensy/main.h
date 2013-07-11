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

#ifndef _MAIN_h
#define _MAIN_h

#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <stdlib.h>

#define CPU_PRESCALE(n)	(CLKPR = 0x80, CLKPR = (n))
#define ADC_Prescaler (_BV(ADPS2) | _BV(ADPS1))
#define ADC_Mux 0

#define TLed PORTD ^= _BV(6)

signed char SameStrPM(uint8_t *is,char *is2,signed char l);
uint8_t FindStr(uint8_t *p,uint16_t pl,char *s,uint8_t l);

uint16_t ADC_Read(uint8_t ch);
#endif