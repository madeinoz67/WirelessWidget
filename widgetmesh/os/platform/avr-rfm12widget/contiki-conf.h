//$Id$//
/***************************************************************************
 Copyright (C) 2009 Stephen Eaton

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License along
 with this program; if not, write to the Free Software Foundation, Inc.,
 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

 Originally written by Stephen Eaton.
 Please post support questions to the WirelessWidget Discussion Group.
 http://code.google.com/p/strobit/
 ***************************************************************************/
/*!

 Created on: 01/10/2009
 Author: Stephen Eaton <seaton@gateway.net.au>

 \file contiki-conf.h
 \ingroup rfm12widget

 \brief ContikiOS configuration file for the RFM12WidgetBoard

 */
/**************************************************************************/
#ifndef __CONTIKI_CONF_H__
#define __CONTIKI_CONF_H__

#include <stdint.h>

/*
 * MCU and clock rate
 */
#define MCU_MHZ 10
#define PLATFORM PLATFORM_AVR

/* Clock ticks per second */
#define CLOCK_CONF_SECOND 200

/* Pre-allocated memory for loadable modules heap space (in bytes)*/
//#define MMEM_CONF_SIZE 256


/* Use the following address for code received via the codeprop
 * facility
 */
//#define EEPROMFS_ADDR_CODEPROP 0x8000

#define CCIF
#define CLIF

typedef unsigned short clock_time_t;
typedef unsigned char u8_t;
typedef unsigned short u16_t;
typedef unsigned long u32_t;
typedef unsigned short uip_stats_t;
typedef unsigned long off_t;
typedef int32_t s32_t;

void clock_delay(unsigned int us2);
void clock_wait(int ms10);
void clock_set_seconds(unsigned long s);
unsigned long clock_seconds(void);

/*
 * Definitions below are dictated by the hardware and not really
 * changeable!
 */
#define LED_GREEN   _BV(PINB0)
#define LED_YELLOW  _BV(PINB1)
#define LED_ALL    (LED_YELLOW | LED_GREEN)



#endif /* __CONTIKI_CONF_H__ */
