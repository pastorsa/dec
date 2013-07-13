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

#ifndef _COM_h
#define _COM_h

#define DBG 0

#if DBG
#define dbg_init  usb_init(); \
  while (!usb_configured()); \
  _delay_ms(1000); \
  dbg_s("\n----- Debug Process Started -----\n")

#define dbg_s(s) print(s)
#define dbg_c(c) pchar(c)
#define dbg_n(n) phex(n)
#define dbg_n16(n) phex16(n)

#include "usb_debug/usb_debug.h"
#include "usb_debug/print.h"
#else
#define dbg_init
#define dbg_s(s)
#define dbg_c(c)
#define dbg_n(n)
#define dbg_n16(n)
#endif

// #define BFSize (unsigned)900

// Uncomment to enable TCP and UDP Checksum Verifications - Hex size +134 bytes -
// #define _UDP_TCP_DCheck

#endif
