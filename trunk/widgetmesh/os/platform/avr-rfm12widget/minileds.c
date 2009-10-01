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

 \file mini-leds.c
 \ingroup rfm12widget

 \brief implementation of minileds module for RFM12WidgetBoard

 */


#include "contiki.h"

#include "dev/leds.h"


#define LED_DDR     DDRB
#define LED_PORT    PORTB

void
leds_init(void)
{
    LED_DDR |= LED_ALL;

    //turn off widget LEDS, HI = off
    LED_PORT |= LED_ALL;
}

void
leds_on(unsigned char leds)
{
    LED_PORT &= ~ leds;     //LOW = ON
}

void
leds_off(unsigned char leds)
{
    LED_PORT |= leds;      //HI = OFF
}

void
leds_toggle(unsigned char leds)
{
  /*
   * Synonym: void leds_invert(unsigned char leds);
   */
  asm(".global leds_invert\nleds_invert:\n");

  LED_PORT ^= leds;
}
