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

#ifndef _ADDRESS_h
#define _ADDRESS_h

// NOTE: Change both, NODE_ID and NODE_ID_HEX !!
#define NODE_ID 1
#define NODE_ID_HEX 0x01

static const uint8_t IPAddr[4] = {10,0,0,NODE_ID};
static uint8_t MACAddr[6] = {0x0,0x0,0x0,0x0,0x0,NODE_ID_HEX};

#endif
