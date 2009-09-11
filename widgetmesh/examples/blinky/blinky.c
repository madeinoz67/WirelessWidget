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
 Please post support questions to the Strobit Discussion Group.
 http://code.google.com/p/strobit/
 ***************************************************************************/
/*!
 $Id: $
 Created on: 13/08/2009
 Author: seaton

 \file blinky.c
 \ingroup

 \brief
  Toggles onboard LEDS

 */
/**************************************************************************/
#include "contiki.h"
#include "dev/leds.h"


static int period = CLOCK_SECOND / 2;

PROCESS(blinker, "Blinker");
AUTOSTART_PROCESSES(&blinker);

PROCESS_THREAD(blinker, ev, data)
{
   static struct etimer et;

   PROCESS_BEGIN();

   etimer_set(&et, period);
   while(1) {
      PROCESS_WAIT_EVENT();
      if(etimer_expired(&et)) {
         /* Toggle LED state */
         leds_invert(LEDS_ALL);
         printf("Is there anybody out there?\n");
         /* Set timer again */
         etimer_set(&et, period);
      }
   }
   PROCESS_END();
}
